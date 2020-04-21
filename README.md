# Model-based Validation of Autonomous Driving Using WPILib

## Abstract

We will implement autonomous driving using WPILib 2020, then simulate it against
a realistic mathematical model of FRC team 3512's 2020 robot.

This is an end-to-end presentation on autonomous driving for differential drive
and skid steer robots using the tools available in WPILib 2020. First, it will
give an overview of the disciplines required like localization (knowing where
you are), motion planning (planning a way to get where you want to go), and
control (actually getting there). Then, these concepts will be implemented and
simulated against a realistic mathematical model of FRC team 3512's 2020 robot.

## Resources

WPILib's documentation and tutorials can be found at
[docs.wpilib.org](https://docs.wpilib.org). The
[trajectory tutorial](https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/index.html)
walks through implementing all of the topics mentioned in the presentation.

For a deeper dive into some of the math from the presentation, refer to the
relevant section in
[Controls Engineering in FRC](https://github.com/calcmogul/controls-engineering-in-frc) (the topics discussed roughly map to the parts listed in the table of
contents).

The unit test presented from FRC team 3512's 2020 robot code is
[here](https://github.com/frc3512/Robot-2020/blob/master/src/test/cpp/DrivetrainControllerTest.cpp).

[FRC Characterization](https://github.com/wpilibsuite/frc-characterization) was
used to obtain feedforward gains for the drivetrain (it supports mechanisms like
elevators and flywheels too). The feedforward gains were converted into a
simulated drivetrain model using the state-space modeling library in
[this fork of WPILib](https://github.com/mcm001/allwpilib/tree/state-space-v2).
It's being prepped for inclusion in WPILib 2021.

If any FIRST robotics team members have questions on the material referenced
here or need help applying it, feel free to ask on the
[FRC Discord](https://discord.gg/frc) in one of the programming channels.
Several WPILib contributors are fairly active there, and complex topics are
easier to teach when there's a feedback loop. :)
