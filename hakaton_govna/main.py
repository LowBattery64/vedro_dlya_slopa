"""Competition application template."""

import time

from service import MaixcamLink

def main():
    link = MaixcamLink()
    link.command.set_enabled("motors")
    link.command.set_all_speeds(0)
    link.start()

    try:
        while link.running:
            telemetry = link.read_telemetry()
            
            time.sleep(0.10)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()


if __name__ == "__main__":
    main()
