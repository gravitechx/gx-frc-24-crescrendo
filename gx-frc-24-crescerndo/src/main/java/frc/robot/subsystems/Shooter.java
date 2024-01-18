package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax motor;

    public Shooter() {
        motor = new CANSparkMax(Constants.Shooter.kMotorPort, MotorType.kBrushless);
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooterspeed", motor.get());
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }


}
