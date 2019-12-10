-- Copyright 2019 kvedder@seas.upenn.edu
-- School of Engineering and Applied Sciences,
-- University of Pennsylvania
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.
-- ========================================================================


sim = {
  kLaserStdDev = 0.015;
  kArcExecStdDev = 0.4;
  kArcReadStdDev = 0.2;
  kRotateExecStdDev = 0.001;
  kRotateReadStdDev = 0.001;
  kStartPositionX = 4;
  kStartPositionY = 0;
  kStartPositionTheta = 0;

  kMap = "./src/ServiceRobotControlStack/control_stack/maps/loop_small_bumps.map";

  laser = {
    min_angle = -math.pi / 2.0,
    max_angle = math.pi / 2.0,
    num_readings = 100,
    min_reading = 0.1,
    max_reading = 5.0,
  };
};
sim.laser.angle_delta = math.abs(sim.laser.max_angle - sim.laser.min_angle) / (sim.laser.num_readings - 1.0);
-- print(math.abs(sim.laser.max_angle - sim.laser.min_angle))
-- print(sim.laser.num_readings - 1.0)
-- print(sim.laser.angle_delta)