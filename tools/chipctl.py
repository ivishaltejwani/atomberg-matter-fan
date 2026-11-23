#!/usr/bin/env python3
"""Pair and test a fan from this Mac with the Matter SDK's own controller (same core as chip-tool).

  pip install home-assistant-chip-core home-assistant-chip-clusters      (into .venv)

  chipctl.py pase QR                 Bluetooth discovery + secure pairing session only (no WiFi needed)
  WIFI_PASS=... chipctl.py commission QR SSID     full pairing over Bluetooth, WiFi handed to the fan
  chipctl.py read                    read the fan's main attributes over WiFi
  chipctl.py test [--spin]           run every command; without --spin only OFF / 0% are sent
  chipctl.py unpair                  remove this controller from the fan

Controller state (its fabric) lives in ~/.matter_fan_chipctl/.  Test PAA roots are downloaded on first use,
because the fan uses the Matter test vendor ID.
"""
import asyncio
import os
import sys
import urllib.request

import chip.native
import chip.CertificateAuthority
import chip.clusters as Clusters
from chip.ChipStack import ChipStack

HOME = os.path.expanduser("~/.matter_fan_chipctl")
PAA = os.path.join(HOME, "paa")
PAA_URL = "https://raw.githubusercontent.com/project-chip/connectedhomeip/master/credentials/development/paa-root-certs/"
NODE = 0x1001
EP = 1


def paa_store():
    os.makedirs(PAA, exist_ok=True)
    for name in ("Chip-Test-PAA-FFF1-Cert.der", "Chip-Test-PAA-NoVID-Cert.der"):
        path = os.path.join(PAA, name)
        if not os.path.exists(path):
            urllib.request.urlretrieve(PAA_URL + name, path)
    return PAA


def controller():
    os.makedirs(HOME, exist_ok=True)
    chip.native.Init()
    stack = ChipStack(persistentStoragePath=os.path.join(HOME, "storage.json"), enableServerInteractions=False)
    cam = chip.CertificateAuthority.CertificateAuthorityManager(stack, stack.GetStorageManager())
    cam.LoadAuthoritiesFromStorage()
    ca = cam.activeCaList[0] if cam.activeCaList else cam.NewCertificateAuthority()
    fa = ca.adminList[0] if ca.adminList else ca.NewFabricAdmin(vendorId=0xFFF1, fabricId=1)
    KEEP.extend([stack, cam, ca, fa])  # their destructors shut the controller down
    return fa.NewController(nodeId=112233, paaTrustStorePath=paa_store())


KEEP = []


def show(label, value):
    print(f"  {label:<28} {value}")


async def read_state(ctrl):
    FC, OO, BI = Clusters.FanControl, Clusters.OnOff, Clusters.BasicInformation
    want = [(0, BI.Attributes.VendorName), (0, BI.Attributes.ProductName), (0, BI.Attributes.SerialNumber),
            (0, BI.Attributes.SoftwareVersionString), (EP, FC.Attributes.FanMode), (EP, FC.Attributes.FanModeSequence),
            (EP, FC.Attributes.PercentSetting), (EP, FC.Attributes.PercentCurrent)]
    res = await ctrl.ReadAttribute(NODE, want)
    out = {}
    for ep, attr in want:
        v = res.get(ep, {}).get(attr.cluster_id and next(c for c in (FC, BI) if c.id == attr.cluster_id), {}).get(attr)
        out[attr.__name__] = v
        show(attr.__qualname__.replace("Attributes.", ""), v)
    return out


async def set_and_check(ctrl, label, attr_value, expect):
    FC = Clusters.FanControl
    await ctrl.WriteAttribute(NODE, [(EP, attr_value)])
    await asyncio.sleep(1.5)
    res = await ctrl.ReadAttribute(NODE, [(EP, FC.Attributes.FanMode), (EP, FC.Attributes.PercentSetting),
                                          (EP, FC.Attributes.PercentCurrent)])
    got = res[EP][FC]
    mode, ps, pc = got[FC.Attributes.FanMode], got[FC.Attributes.PercentSetting], got[FC.Attributes.PercentCurrent]
    ok = expect(mode, ps, pc)
    print(f"  {'PASS' if ok else 'FAIL'}  {label:<34} -> mode={int(mode)} setting={ps} current={pc}")
    return ok


async def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    cmd = args[0]
    ctrl = controller()
    FC = Clusters.FanControl
    try:
        if cmd in ("pase", "commission"):
            from chip.setup_payload import SetupPayload
            p = SetupPayload().ParseQrCode(args[1])
            disc, pin = int(p.long_discriminator), int(p.setup_passcode)
            if cmd == "pase":  # Bluetooth only: no DNS-SD, so it also works without Local Network access
                await ctrl.EstablishPASESessionBLE(pin, disc, NODE)
                print("PASE session established over Bluetooth")
            else:
                node = await ctrl.CommissionWiFi(disc, pin, NODE, args[2], os.environ["WIFI_PASS"])
                print(f"commissioned as node 0x{node:X}")
        elif cmd == "read":
            await read_state(ctrl)
        elif cmd == "unpair":
            await ctrl.UnpairDevice(NODE)
            print("controller removed from the fan")
        elif cmd == "test":
            spin = "--spin" in args
            print("read:")
            await read_state(ctrl)
            results = []
            print("commands:")
            results.append(await set_and_check(ctrl, "FanMode = Off", FC.Attributes.FanMode(FC.Enums.FanModeEnum.kOff),
                                               lambda m, s, c: m == 0 and c == 0))
            results.append(await set_and_check(ctrl, "PercentSetting = 0", FC.Attributes.PercentSetting(0),
                                               lambda m, s, c: m == 0 and c == 0))
            if spin:
                results.append(await set_and_check(ctrl, "FanMode = On (starts at speed 1)",
                                                   FC.Attributes.FanMode(FC.Enums.FanModeEnum.kOn),
                                                   lambda m, s, c: m != 0 and c > 0))
                for pct in (33, 50, 100, 16):
                    results.append(await set_and_check(ctrl, f"PercentSetting = {pct}", FC.Attributes.PercentSetting(pct),
                                                       lambda m, s, c, p=pct: m != 0 and s == p and c == p))
                results.append(await set_and_check(ctrl, "PercentSetting = 0 (turns off)", FC.Attributes.PercentSetting(0),
                                                   lambda m, s, c: m == 0 and c == 0))
                results.append(await set_and_check(ctrl, "FanMode = High", FC.Attributes.FanMode(FC.Enums.FanModeEnum.kHigh),
                                                   lambda m, s, c: m != 0 and c > 0))
                results.append(await set_and_check(ctrl, "FanMode = Off", FC.Attributes.FanMode(FC.Enums.FanModeEnum.kOff),
                                                   lambda m, s, c: m == 0 and c == 0))
            print(f"{sum(results)}/{len(results)} passed" + ("" if spin else "  (motor commands skipped: pass --spin)"))
        else:
            sys.exit(__doc__)
    finally:
        ctrl.Shutdown()


if __name__ == "__main__":
    asyncio.run(main())
