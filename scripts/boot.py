"""
*
* \brief boot.py
*
* Copyright (c) 2019 Microchip Technology Inc.
                2025 Angus Logan (awjlogan@gmail.com)
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the Licence at
*
 * http://www.apache.org/licenses/LICENSE-2.0
*
 * Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
"""

#!/usr/bin/env python

import sys
import time
import serial
import serial.tools.list_ports
import optparse

from rich.progress import Progress

# ------------------------------------------------------------------------------
# Bootloader commands
BL_CMD_UNLOCK = 0xA0
BL_CMD_DATA = 0xA1
BL_CMD_VERIFY = 0xA2
BL_CMD_RESET = 0xA3

# Bootloader responses
BL_RESP_OK = 0x50
BL_RESP_ERROR = 0x51
BL_RESP_INVALID = 0x52
BL_RESP_CRC_OK = 0x53
BL_RESP_CRC_FAIL = 0x54

BOOTLOADER_SIZE = 1024


# ------------------------------------------------------------------------------
def error(text):
    sys.stderr.write(f"Error: {text}\n")
    sys.exit(1)


# ------------------------------------------------------------------------------
def warning(text):
    sys.stderr.write("Warning: {text}\n")


# ------------------------------------------------------------------------------
def verbose(verb, text, nl=True):
    if verb:
        if nl:
            print(text)
        else:
            print(text, end="")


# ------------------------------------------------------------------------------
def crc32_tab_gen():
    res = []

    for i in range(256):
        value = i

        for j in range(8):
            if value & 1:
                value = (value >> 1) ^ 0xEDB88320
            else:
                value = value >> 1

        res += [value]

    return res


# ------------------------------------------------------------------------------
def crc32(tab, data):
    crc = 0xFFFFFFFF

    for d in data:
        crc = tab[(crc ^ d) & 0xFF] ^ (crc >> 8)

    return crc


# ------------------------------------------------------------------------------
def uint32(v):
    return [(v >> 0) & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]


# ------------------------------------------------------------------------------
def get_response(port):
    v = port.read()

    if len(v) == 0:
        return None
    elif len(v) > 1:
        error("Invalid response received (size > 1)")

    return v[0]


# ------------------------------------------------------------------------------
def send_request(port, cmd, data):
    req = [cmd] + uint32(0x2B620BC3) + data

    for i in range(3):
        port.write(bytes(req))
        resp = get_response(port)

        if resp is None:
            warning(f"No response received, retrying {(i + 1)}")
            time.sleep(0.2)
        else:
            return resp

    error("No response received, giving up")


# ------------------------------------------------------------------------------
def upload(options, port, offset):
    try:
        data = data = [x for x in open(options.file, "rb").read()]
    except Exception as inst:
        error(inst)

    while len(data) % 256 > 0:
        data += [0xFF]

    crc32_tab = crc32_tab_gen()
    crc = crc32(crc32_tab, data)

    size = len(data)

    verbose(options.verbose, "Unlocking")
    resp = send_request(port, BL_CMD_UNLOCK, uint32(offset) + uint32(size))

    if resp != BL_RESP_OK:
        error(
            "Invalid response code (0x%02x). Check that your file size and offset are correct."
            % resp
        )

    blocks = [data[i : i + 256] for i in range(0, len(data), 256)]

    verbose(
        options.verbose,
        "Uploading %d blocks at offset %d (0x%x)" % (len(blocks), offset, offset),
    )

    addr = offset

    if options.verbose:
        with Progress() as p:
            t = p.add_task("Uploading...", total=len(blocks))
            while not p.finished and resp == BL_RESP_OK:
                for idx, blk in enumerate(blocks):
                    p.update(t, advance=1)
                    resp = send_request(port, BL_CMD_DATA, uint32(addr) + blk)
                    addr += 256

                    if resp != BL_RESP_OK:
                        break

        if resp != BL_RESP_OK:
            error("Invalid response code (0x%02x)" % resp)

    else:
        for idx, blk in enumerate(blocks):
            resp = send_request(port, BL_CMD_DATA, uint32(addr) + blk)
            addr += 256

            if resp != BL_RESP_OK:
                error("Invalid response code (0x%02x)" % resp)

    verbose(options.verbose, "Verification", nl=False)
    resp = send_request(port, BL_CMD_VERIFY, uint32(crc))

    if resp == BL_RESP_CRC_OK:
        verbose(options.verbose, "... success")
    else:
        error("... fail (status = 0x%02x)" % resp)


# ------------------------------------------------------------------------------
def main():
    parser = optparse.OptionParser(
        prog="boot.py",
        description="This program loads a binary file through UART to a SAML10 with bootloader.",
        epilog="github.com/awjlogan/bootloader_uart_saml10",
    )
    parser.add_option(
        "-v",
        "--verbose",
        dest="verbose",
        help="enable verbose output",
        default=False,
        action="store_true",
    )
    parser.add_option(
        "-i", "--interface", dest="port", help="Communication interface", metavar="PATH"
    )
    parser.add_option(
        "-f", "--file", dest="file", help="Binary file to program", metavar="FILE"
    )
    parser.add_option(
        "-o",
        "--offset",
        dest="offset",
        help="Destination offset (default 0x400)",
        default="0x400",
        metavar="OFFS",
    )
    parser.add_option(
        "-r",
        "--reboot",
        dest="reboot",
        help="send the reboot command",
        default=False,
        action="store_true",
    )
    parser.add_option(
        "",
        "--boot",
        dest="boot",
        help="Enable write to the bootloader area",
        default=False,
        action="store_true",
    )
    (options, args) = parser.parse_args()

    if not options.port:
        print("Available ports:")
        for comport in serial.tools.list_ports.comports():
            print("  - ", comport.device)
        error("Communication port is required.")

    try:
        offset = int(options.offset, 0)
    except ValueError:
        error("Invalid offset value: %s" % options.offset)

    if (offset < BOOTLOADER_SIZE) and not options.boot:
        error(
            "Offset is within the bootloader area, use --boot options to unlock writes"
        )

    try:
        port = serial.Serial(options.port, 115200, timeout=1)
    except serial.serialutil.SerialException as inst:
        error(inst)
    else:
        with port:
            if options.file is not None:
                upload(options, port, offset)
            if options.reboot:
                verbose(options.verbose, "Rebooting")
                send_request(port, BL_CMD_RESET, uint32(0) * 4)

        verbose(options.verbose, "Done!")


# ------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
