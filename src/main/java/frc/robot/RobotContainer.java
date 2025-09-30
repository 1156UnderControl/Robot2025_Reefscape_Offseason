package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightScore;
import frc.robot.commands.Intake.IntakeExpellCoral;
import frc.robot.commands.Intake.OverrideCoralMode;
import frc.robot.commands.Intake.StopCollecting;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.commands.States.CollectCoralPosition;
import frc.robot.commands.States.DefaultPosition;
import frc.robot.commands.States.ScoreObjectPosition;
import frc.robot.commands.States.AlignToClimb;
import frc.robot.commands.States.AutoScoreCoralPosition;
import frc.robot.commands.States.CollectAlgaePosition;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
    private final OperatorController keyboard;
    private final DriverController driverController;
  
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ScorerSubsystem scorer;
    private final ClimberSubsystem climber;
  
    private SwerveModuleConstants[] modulesArray = SwerveConstants.getModuleConstants();
  
    public RobotContainer() {
      this.scorer = ScorerSubsystem.getInstance();
      this.intake = IntakeSubsystem.getInstance();
      this.swerve = new SwerveSubsystem(this.scorer.getTargetCoralReefLevelSupplier(), this.scorer.getTargetAlgaeReefLevelSupplier(), SwerveConstants.getSwerveDrivetrainConstants(),
        modulesArray[0], modulesArray[1], modulesArray[2], modulesArray[3]);
      this.climber = ClimberSubsystem.getInstance();
     
      this.driverController = DriverController.getInstance();
      this.keyboard = OperatorController.getInstance();

      this.scorer.setIntakeUpSupplier(this.intake.getIntakeUpSupplier());
      this.swerve.setDefaultCommand(Commands.run(() -> swerve.driveAlignAngleJoystick(), this.swerve));
      this.scorer.setDefaultCommand(new DefaultPosition(intake, scorer));
      this.climber.setDefaultCommand(Commands.run(() -> this.climber.goToDefaultPosition(), climber));
      this.configureButtonBindings();
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {

    this.driverController.a().onTrue(
      new CollectCoralFromIndexer(scorer, intake)
    ).and(() -> !scorer.hasObject() && intake.indexerHasCoral());
    
    this.driverController.x().onTrue(
      new CollectCoralPosition(intake)
    ).and(() -> !intake.indexerHasCoral());

    this.driverController.b().onTrue(
      new StopCollecting(intake)
    );

    this.driverController.y().whileTrue(
      new IntakeExpellCoral(intake)
    );

    this.keyboard.prepareToScore().and(() -> scorer.hasObject()).onTrue(
      new ScoreObjectPosition(scorer, keyboard)
    );

    this.keyboard.cancelAction().onTrue(
      new DefaultPosition(intake, scorer)
    );
  
    this.keyboard.removeAlgaeFromBranch().onTrue(
      new CollectAlgaePosition(scorer, keyboard)
    );

    this.keyboard.reefL1()
    .onTrue(new InstantCommand(() -> {
      this.scorer.setTargetCoralLevel(ReefLevel.L1);
      this.scorer.setTargetAlgaeLevel(AlgaeHeightReef.GROUND);
      this.scorer.setTargetAlgaeLevelToScore(AlgaeHeightScore.PROCESSOR);
    }));

    this.keyboard.reefL2()
    .onTrue(new InstantCommand(() -> {
      this.scorer.setTargetCoralLevel(ReefLevel.L2);
      this.scorer.setTargetAlgaeLevel(AlgaeHeightReef.LOW);
    }));

    this.keyboard.reefL3()
    .onTrue(new InstantCommand(() -> {
      this.scorer.setTargetCoralLevel(ReefLevel.L3);
      this.scorer.setTargetAlgaeLevel(AlgaeHeightReef.MID);
      ;
    }));

    this.keyboard.reefL4().onTrue(
      new InstantCommand(() -> {
         this.scorer.setTargetCoralLevel(ReefLevel.L4);
         this.scorer.setTargetAlgaeLevelToScore(AlgaeHeightScore.NET);
      }
    ));
    
    this.keyboard.alignToClimb().onTrue(
      new AlignToClimb(climber, swerve, scorer)
    );
    
    bindAutoScoreCommands();

    //driverController.x().and(() -> DriverStation.isDisabled()).whileTrue(Commands
    //    .runEnd(() -> new CoastState(scorer, intake), () -> new BrakeState(scorer, intake)).ignoringDisable(true));

    //driverController.y().and(() -> DriverStation.isDisabled()).whileTrue(new UpdatePivotInternalEncoder(scorer).ignoringDisable(true));
      
  }

  private void bindAutoScoreCommands() {
    this.keyboard.goToReefA().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.A));
    NamedCommands.registerCommand("Auto Score Coral A", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.A));

    this.keyboard.goToReefB().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.B));
    NamedCommands.registerCommand("Auto Score Coral B", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.B));

    this.keyboard.goToReefC().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.C));
    NamedCommands.registerCommand("Auto Score Coral C", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.C));

    this.keyboard.goToReefD().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.D));
    NamedCommands.registerCommand("Auto Score Coral D", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.D));

    this.keyboard.goToReefE().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.E));
    NamedCommands.registerCommand("Auto Score Coral E", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.E));

    this.keyboard.goToReefF().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.F));
    NamedCommands.registerCommand("Auto Score Coral F", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.F));

    this.keyboard.goToReefG().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.G));
    NamedCommands.registerCommand("Auto Score Coral G", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.G));

    this.keyboard.goToReefH().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.H));
    NamedCommands.registerCommand("Auto Score Coral H", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.H));

    this.keyboard.goToReefI().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.I));
    NamedCommands.registerCommand("Auto Score Coral I", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.I));

    this.keyboard.goToReefJ().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.J));
    NamedCommands.registerCommand("Auto Score Coral J", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.J));

    this.keyboard.goToReefK().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.K));
    NamedCommands.registerCommand("Auto Score Coral K", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.K));

    this.keyboard.goToReefL().onTrue(new AutoScoreCoralPosition(scorer, swerve, TargetBranch.L));
    NamedCommands.registerCommand("Auto Score Coral L", new AutoScoreCoralPosition(scorer, swerve, TargetBranch.L));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}