"""
Injection-only test program for the 9-chip A-STEP bench.
"""

import argparse
import asyncio
import binascii
import logging
import os
import sys
import time

import pandas as pd

import drivers.boards
from drivers.astropix.decode import Decode


async def buffer_flush(boardDriver, layerlst=range(3)):
    """Flush SPI lanes and FPGA buffer before an injection run."""
    logger.info("Flush chips before injection data collection")
    await boardDriver.holdLayers(hold=False, flush=True)
    for layer in layerlst:
        interrupt_counter = 0
        interrupt = await boardDriver.getLayerStatus(layer)
        while interrupt & 1 == 0 and interrupt_counter < 20:
            logger.info("interrupt low")
            await boardDriver.layersSelectSPI(flush=True)
            await boardDriver.writeSPIBytesToLane(lane=layer, bytes=[0x00] * 128)
            await boardDriver.layersDeselectSPI(flush=True)
            interrupt_counter += 1
            interrupt = await boardDriver.getLayerStatus(layer)
    await boardDriver.holdLayers(hold=True, flush=True)
    logger.info("Flush FPGA buffer before injection data collection")
    await boardDriver.readoutReadBytes(4098)
    await boardDriver.resetLayerStatCounters(layer)


async def get_readout(boardDriver, counts: int = 4096):
    buffer_size = await boardDriver.readoutGetBufferSize()
    readout = await boardDriver.readoutReadBytes(counts)
    return buffer_size, readout


async def getBuffer(boardDriver):
    buffer_size = await boardDriver.readoutGetBufferSize()
    readout = await boardDriver.readoutReadBytes(buffer_size)
    return buffer_size, readout


def dataParse_autoread(data_lst, buffer_lst, bitfile: str = None):
    all_data = b""
    for i, buff in enumerate(buffer_lst):
        if buff > 0:
            readout_data = data_lst[i][:buff]
            all_data += readout_data
            if bitfile:
                bitfile.write(f"{str(binascii.hexlify(readout_data))}\n")
    return all_data


async def printStatus(boardDriver, time=0.0, buff=0):
    status = [await boardDriver.getLayerStatus(layer) for layer in range(3)]
    ctrl = [await boardDriver.getLayerControl(layer) for layer in range(3)]
    wrongl = [await boardDriver.getLayerWrongLength(layer) for layer in range(3)]
    logger.info(
        "[{time:04.2} s] buff={0:04d} status: 0={1[0]:02b}-{2[0]:06b}-{3[0]:04d} 1={1[1]:02b}-{2[1]:06b}-{3[1]:04d} 2={1[2]:02b}-{2[2]:06b}-{3[2]:04d}".format(
            buff, status, ctrl, wrongl, time=time
        )
    )


async def main(args):
    if args.inject is None:
        raise ValueError("--inject is required for injection-only mode.")

    boardDriver = drivers.boards.getCMODUartDriver("/dev/ttyUSB1")
    logger.debug(f"boardDriver instanciated: {boardDriver}")
    await boardDriver.open()
    logger.info("Opened FPGA, testing...")
    try:
        fwid = await boardDriver.readFirmwareID()
        logger.debug(f"FW ID: {fwid}")
    except Exception as exc:
        raise RuntimeError("Could not read or write from astropix!") from exc

    ok = await boardDriver.setExternalClock(
        enable=False,
        ext_clock_is_differential=False,
        waitForClockChange=True,
    )
    logger.info(f"External clock switch result: {ok}")

    await boardDriver.enableSensorClocks(flush=True)
    await boardDriver.layersConfigFPGATimestampFrequency(
        targetFrequencyHz=40000000,
        flush=True,
    )
    await boardDriver.layersConfigFPGATimestamp(
        enable=True,
        forced_value=False,
        use_divider=True,
        use_tlu=False,
        flush=True,
    )
    await boardDriver.ioSetAstropixTSToFPGATS(True)

    await boardDriver.configureLayerSPIDivider(10, flush=True)
    await boardDriver.rfg.write_layers_cfg_nodata_continue(value=8, flush=True)

    pathdelim = os.path.sep
    ymlpath = [
        os.getcwd() + pathdelim + "scripts" + pathdelim + "config" + pathdelim + y + ".yml"
        for y in args.yaml
    ]

    try:
        for layer, (nchips, yml) in enumerate(zip(args.chipsPerRow, ymlpath)):
            boardDriver.setupASIC(
                version=3,
                lane=layer,
                chipsPerLane=nchips,
                configFile=yml,
            )
    except FileNotFoundError as exc:
        logger.error(
            f"Config File {ymlpath} was not found, pass the name of a config file from the scripts/config folder"
        )
        raise exc

    layer_index = args.inject[0] - 1
    chip_index = args.inject[1]
    row_index = args.inject[2]
    col_index = args.inject[3]

    if not (0 <= layer_index <= 2):
        raise ValueError(f"Injection layer {layer_index} out of range [0, 2].")
    if not (0 <= chip_index <= args.chipsPerRow[layer_index] - 1):
        raise ValueError(f"Injection chip {chip_index} out of range [0, {args.chipsPerRow[layer_index] - 1}].")
    if row_index < 0 or col_index < 0:
        raise ValueError("Injection row and column must be non-negative.")


    for chip in range(args.chipsPerRow[layer_index]):
        for row in range(35):
            for col in range(35):
                boardDriver.asics[layer_index].disable_pixel(chip, col, row)

    boardDriver.asics[layer_index].enable_inj_col(chip_index, col_index)
    boardDriver.asics[layer_index].enable_inj_row(chip_index, row_index)
    boardDriver.asics[layer_index].enable_pixel(
        chip=chip_index,
        col=col_index,
        row=row_index,
    )

    if args.vinj is not None:
        boardDriver.asics[layer_index].asic_config[f"config_{chip_index}"]["vdacs"]["vinj"][1] = int(
            args.vinj / 1000 * 1024 / 1.8
        )

    injector = boardDriver.getInjector()
    injector.setPattern(40, 3, 100, 0, 1)
    #default: period=100, clkdiv=300, initdelay=100, cycle=0, ppset=1
    await boardDriver.ioSetInjectionToChip(enable=True, flush=True)

    await printStatus(boardDriver)
    for layer in range(3):
        await boardDriver.zeroLayerWrongLength(layer, flush=True)

    layerlst = range(len(args.yaml))
    await boardDriver.disableLayersReadout(flush=True)
    await boardDriver.resetLayersFull()

    await boardDriver.layersSelectSPI(flush=True)
    for layer in layerlst:
        await boardDriver.writeRoutingFrame(lane=layer, firstChipID=0)
    await boardDriver.layersDeselectSPI(flush=True)

    for i in range(args.chipsPerRow[layer_index]):
        await boardDriver.layersSelectSPI(flush=True)
        for layer in layerlst:
            if i < args.chipsPerRow[layer]:
                await boardDriver.writeSPIAsicConfig(lane=layer, targetChip=i)
        await boardDriver.layersDeselectSPI(flush=True)

    await buffer_flush(boardDriver, layerlst)

    data_stream_lst = []
    buffer_length_lst = []
    ofile = open(output_path + ".bin", "wb")
    if args.runTime is not None:
        end_time = time.time() + args.runTime
    else:
        end_time = float("inf")

    await boardDriver.enableLayersReadout(layerlst, autoread=not args.noAutoread, flush=True)
    await injector.start()

    run = time.time() < end_time
    while run:
        try:
            if args.noAutoread:
                for layer in layerlst:
                    await boardDriver.writeSPIBytesToLane(lane=layer, bytes=[0x00] * 255)

            if args.readout is None:
                task = asyncio.create_task(getBuffer(boardDriver))
            else:
                task = asyncio.create_task(get_readout(boardDriver, args.readout))
            await task
            buff, readout = task.result()
            data_stream_lst.append(readout)
            buffer_length_lst.append(buff)
            if buff > 0:
                ofile.write(readout[:buff])
            await printStatus(boardDriver, time.time() - end_time, buff=buff)
            print(f"  {buff:04d}  ", end="\r")
            run = time.time() < end_time
        except (KeyboardInterrupt, asyncio.CancelledError):
            logger.info("[Ctrl+C] while in injection loop - exiting.")
            run = False

    await printStatus(boardDriver, time.time() - end_time)
    await boardDriver.disableLayersReadout(flush=True)
    await injector.stop()
    await boardDriver.close()
    ofile.close()

    data_stream = dataParse_autoread(data_stream_lst, buffer_length_lst, None)
    dec = Decode()
    df = dec.decode_readout(logger, data_stream, i=0, printer=False)
    if len(df) > 0:
        csvframe = [
            "readout",
            "layer",
            "chipID",
            "payload",
            "location",
            "isCol",
            "timestamp",
            "tot_msb",
            "tot_lsb",
            "tot_total",
            "tot_us",
            "fpga_ts",
        ]
        df.columns = csvframe
        df.to_csv(output_path + ".csv")
    else:
        logger.warning("No injection data written to disk because none were received.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Injection-only A-STEP benchmark script for 9-chip modules.",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    parser.add_argument(
        "-n",
        "--name",
        default="Injection",
        required=False,
        help="Optional suffix appended to output filenames.",
    )
    parser.add_argument(
        "-s",
        "--suffix",
        type=str,
        required=False,
        default=None,
        help="Suffix for output files."
    )
    parser.add_argument(
        "-L",
        "--loglevel",
        type=str,
        choices=["D", "I", "E", "W", "C"],
        default="I",
        help="Set log level.",
    )
    parser.add_argument(
        "-T",
        "--runTime",
        type=float,
        default=None,
        help="Maximum run time in seconds.",
    )
    parser.add_argument(
        "-r",
        "--readout",
        default=0,
        type=int,
        help="Bytes to read per FPGA readout.",
    )
    parser.add_argument(
        "-y",
        "--yaml",
        action="store",
        required=False,
        type=str,
        default=["module7"],
        nargs="+",
        help="YAML config file(s) to load, one per layer.",
    )
    parser.add_argument(
        "-c",
        "--chipsPerRow",
        action="store",
        required=False,
        type=int,
        default=[9],
        nargs="+",
        help="Number of chips per layer.",
    )
    parser.add_argument(
        "-na",
        "--noAutoread",
        action="store_true",
        required=False,
        help="Disable autoread mode.",
    )
    parser.add_argument(
        "-i",
        "--inject",
        action="store",
        default=None,
        type=int,
        nargs=4,
        help="Injection target: layer chip row column.",
    )
    parser.add_argument(
        "-v",
        "--vinj",
        action="store",
        default=None,
        type=int,
        help="Injection voltage in mV.",
    )

    args = parser.parse_args()

    date_dir = time.strftime("%Y%m%d")
    
    layer, chip, row, col = args.inject
    vinj = args.vinj if args.vinj is not None else 0
    base_name = f"{args.name}_{layer}_{chip}_c{col}r{row}_v{vinj}"
    time_info = time.strftime("%H%M%S")
    if args.suffix is not None:
        base_name += f"_{args.suffix}"
    else:
        base_name += f"_{time_info}"
    
    output_dir = os.path.abspath(os.path.join(os.getcwd(), "..", "..", "pretest", date_dir))
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, base_name)

    ll = args.loglevel
    if ll == "D":
        loglevel = logging.DEBUG
    elif ll == "I":
        loglevel = logging.INFO
    elif ll == "E":
        loglevel = logging.ERROR
    elif ll == "W":
        loglevel = logging.WARNING
    elif ll == "C":
        loglevel = logging.CRITICAL
    else:
        loglevel = logging.INFO

    logname = output_path + "_run.log"
    formatter = logging.Formatter("%(asctime)s:%(msecs)d.%(name)s.%(levelname)s:%(message)s")
    os.makedirs(os.path.dirname(os.path.abspath(logname)), exist_ok=True)

    fh = logging.FileHandler(logname)
    fh.setFormatter(formatter)
    sh = logging.StreamHandler()
    sh.setFormatter(formatter)
    logging.getLogger().addHandler(sh)
    logging.getLogger().addHandler(fh)
    logging.getLogger().setLevel(loglevel)

    global logger
    logger = logging.getLogger(__name__)

    if len(args.yaml) > len(args.chipsPerRow):
        args.chipsPerRow = [args.chipsPerRow[0]] * len(args.yaml)
    elif len(args.yaml) < len(args.chipsPerRow):
        raise ValueError("Need one YAML config per chipsPerRow entry.")

    if args.inject is not None and len(args.inject) != 4:
        raise ValueError("--inject requires 4 values: layer chip row column")

    if args.readout == 0:
        args.readout = None
    elif args.readout < 0 or args.readout > 4098:
        args.readout = 4096

    try:
        asyncio.run(main(args))
        logger.info("Finished injection-only run")
    except KeyboardInterrupt:
        logger.info("Stopping due to CTRL-C")
        sys.exit(-1)
    except Exception as exc:
        logger.error(f"Error during injection main: {exc}")
        raise
