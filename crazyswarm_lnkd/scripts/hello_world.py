"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    cf.setParam("motion/disable", 1)

    cf.goTo([0.0, 0.0, 1.5], 0, 2.5)
    timeHelper.sleep(2.8)

    cf.goTo([5.0, 0, 0], 0, 3.5)
    timeHelper.sleep(5.0)

    cf.goTo([0.0, 0.0, -1.2], 0, 3.0)
    timeHelper.sleep(3.5)

    input("enter to end")
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
