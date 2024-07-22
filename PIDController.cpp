#include "PIDController.h"
#include <utility>
#include "Algo/Accumulate.h"
#include <vector>
#include <queue>
#include "Math/UnrealMathUtility.h"

//void UPIDController::EnqueueAndMaintainSize(std::queue<FVector>& Queue, FVector Element, int32& CurrentSize, int32 MaxSize, FVector& SumBuffer)
//{
//    // Add the new element to the queue
//    Queue.push(Element);
//    // Add the new element's value to the sum buffer
//    SumBuffer += Element;
//    // Increment the current size of the queue
//    CurrentSize++;
//
//    // Check if the queue size has exceeded the maximum allowed size
//    if (CurrentSize > MaxSize)
//    {
//        // If it has, remove the oldest element from the queue
//        FVector RemovedElement = Queue.front();
//        Queue.pop();
//        // Subtract the removed element's value from the sum buffer
//        SumBuffer -= RemovedElement;
//        // Decrement the current size of the queue
//        CurrentSize--;
//    }
//}
//Returns FVector4
FPIDResult UPIDController::UpdatePID(const FPIDConfig& Config, FPIDState& State, FVector PositionError, FVector VelocityError, FRotator DroneOrientation, FVector CurrentAngVel, FVector DroneInertiaTensor, float Mass, float Grav, float DeltaTime)
{
    FVector DroneOrientationVector = FVector(DroneOrientation.Roll, DroneOrientation.Pitch, DroneOrientation.Yaw);
    const float CT = 0.000010546f;
    float gravity = 9.8f;
    float length = 0.25f;
    float b_drag = 1e-9f;
    // Position Controller
    FVector PTerm = Config.ProportionalGain * PositionError;    // k1*e

    UE_LOG(LogTemp, Log, TEXT(" Velocity Error: %f, %f, %f"), VelocityError.X, VelocityError.Y, VelocityError.Z);

    State.ControllerIntegralTerm += DeltaTime * PositionError;
    FVector ITerm = Config.IntegralGain * State.ControllerIntegralTerm; // k3*int(e)

    //FVector ITerm = Config.IntegralGain * State.Pos_ControllerSumBuffer; // k3*int(e) from buffer
    FVector DTerm = Config.DerivativeGain * VelocityError;  // k2*VelocityError as the derivative term

    FVector Des_Acc = PTerm + ITerm + DTerm;

    float mag_acc = Des_Acc.Size();
    if (mag_acc == 0.0f) {
        mag_acc = 1.0f; // Prevent divide by zero
    }

    UE_LOG(LogTemp, Log, TEXT("Acceleration magnitude: %f"), mag_acc);

    float U1 = (Mass * Des_Acc.X) / 100;
    float U2 = (Mass * Des_Acc.Y) / 100;
    float U3 = (Mass * Des_Acc.Z) / 100;

    float U1_max = FMath::Max((Mass * Des_Acc.X) / 100, 0);
    float U2_max = FMath::Max((Mass * Des_Acc.Y) / 100, 0);
    float U3_max = FMath::Max((Mass * Des_Acc.Z) / 100, 0);

    float weight =  Mass * gravity;
    //float Des_Thrust = FMath::Sqrt(FMath::Pow(U1_max, 2) + FMath::Pow(U2_max, 2) + FMath::Pow(U3_max + weight, 2));
    float Des_Thrust = U3_max + weight;
    float WeightRPM = FMath::Sqrt(Des_Thrust / CT) * 7.5 / PI;

    UE_LOG(LogTemp, Log, TEXT("weight: %f"), weight);
    UE_LOG(LogTemp, Log, TEXT("Des_Thrust: %f"), Des_Thrust);

    // Defining DronOrientation, Ang Error, and Ang velocity Error
    
    // Finding Desired Attitude
    FVector Angle_Des;
    //Angle_Des.X = FMath::Atan2(U1, U3);
    //Angle_Des.Y = FMath::Atan2(-U2 * FMath::Cos(DroneOrientation.Pitch), U3);
    //Angle_Des.X = FMath::Atan2(U1_max, U3_max);
    //Angle_Des.Y = FMath::Atan2(-U2_max * FMath::Cos(DroneOrientation.Pitch), U3_max);
    Angle_Des.X = FMath::Asin(-Des_Acc.Y / mag_acc / FMath::Cos(DroneOrientation.Pitch));
    Angle_Des.Y = FMath::Asin(Des_Acc.X / mag_acc);
    Angle_Des.Z = 0.0f;

    UE_LOG(LogTemp, Log, TEXT("Angle Des: %f"), Des_Thrust);

    //Making sure the angle doesnt surpass ~15 degrees
    const float MaxAngleDegrees = 15.0f;
    const float MaxAngleRadians = FMath::DegreesToRadians(MaxAngleDegrees);

    Angle_Des.X = FMath::ClampAngle(Angle_Des.X, -MaxAngleRadians, MaxAngleRadians);
    Angle_Des.Y = FMath::ClampAngle(Angle_Des.Y, -MaxAngleRadians, MaxAngleRadians  );
    Angle_Des.Z = FMath::ClampAngle(Angle_Des.Z, -MaxAngleRadians, MaxAngleRadians);

    UE_LOG(LogTemp, Log, TEXT("Clamped Angle Des: %s"), *Angle_Des.ToString());

    const float RollInertia = DroneInertiaTensor.X / 100;  // Ixx in kg*cm^2
    const float PitchInertia =  DroneInertiaTensor.Y / 100;  // Iyy in kg*cm^2
    const float YawInertia = DroneInertiaTensor.Z / 100;    // Izz in kg*cm^2

    FVector AngleError = Angle_Des - DroneOrientationVector;
    // Angular Velocity Controller
    FVector AnglePTerm = Config.AngleProportionalGain * AngleError;
    FVector AngularVelError = -CurrentAngVel;
    //EnqueueAndMaintainSize(AngleControllerBuffer, AngularVelError, State.AngleControllerCurQueueSize, Config.AngleBuffer_Size, State.Angle_ControllerSumBuffer);

    State.AngleIntegralTerm += AngleError * DeltaTime;
    FVector AngleITerm = Config.AngleIntegralGain * State.AngleIntegralTerm;
    FVector AngleDTerm = Config.AngleDerivativeGain * AngularVelError;
    FVector Ang_Acc = AnglePTerm + AngleITerm + AngleDTerm;

    // Each component of the Angle PID controller is multiplied with its respective rotational moment
    float RollTorque = Ang_Acc.Y * RollInertia;
    float PitchTorque = Ang_Acc.X * PitchInertia;
    float YawTorque = Ang_Acc.Z * YawInertia;
    
    float Pitch = PitchTorque / (4); //* CT * length);
    UE_LOG(LogTemp, Log, TEXT("Pitch: %f"), Pitch);

    // Motors Set
    WeightRPM = FMath::Clamp(WeightRPM, 0.0, 10467.0);
    UE_LOG(LogTemp, Log, TEXT("Weight RPM: %f"), WeightRPM);

    FVector4 MotorRPMs;
    MotorRPMs.X = WeightRPM - Pitch;//  - RollTorque / (4) - YawTorque / (4);
    MotorRPMs.Y = WeightRPM - Pitch;//  + RollTorque / (4) + YawTorque / (4);
    MotorRPMs.Z = WeightRPM + Pitch;//  + RollTorque / (4) - YawTorque / (4);
    MotorRPMs.W = WeightRPM + Pitch;//  - RollTorque / (4) + YawTorque / (4);

    MotorRPMs.X = FMath::Clamp(MotorRPMs.X, 0.0, 10467.0);
    MotorRPMs.Y = FMath::Clamp(MotorRPMs.Y, 0.0, 10467.0);
    MotorRPMs.Z = FMath::Clamp(MotorRPMs.Z, 0.0, 10467.0);
    MotorRPMs.W = FMath::Clamp(MotorRPMs.W, 0.0, 10467.0);

    // Create the result struct and return it
    FPIDResult Result;
    Result.MotorRPMs = MotorRPMs;
    //Result.AngleError = AngleError;

    Log_Messages(RollTorque, PitchTorque, YawTorque,VelocityError,U1,U2,U3,MotorRPMs, Angle_Des, AngleError, AngularVelError, CurrentAngVel, Des_Acc, AngleITerm, AnglePTerm, AngleDTerm, PTerm, ITerm, DTerm);

    return Result;
}


void UPIDController::Log_Messages(float Roll_Torque, float Pitch_Torque, float Yaw_Torque, FVector VelSetpoint,float U_1,float U_2,float U_3,FVector4 Motor_RPMs, FVector Des_Angle, FVector Angle_Error,FVector AngleVel_Error,FVector CurrentVel,FVector DesAcc,FVector AngleITerm,FVector AnglePTerm,FVector AngleDTerm, FVector PTerm, FVector ITerm, FVector DTerm){

    // Logging the calculated torques for debugging purposes
    UE_LOG(LogTemp, Log, TEXT("Roll Torque: %f"), Roll_Torque);
    UE_LOG(LogTemp, Log, TEXT("Pitch Torque: %f"), Pitch_Torque);
    UE_LOG(LogTemp, Log, TEXT("Yaw Torque: %f"), Yaw_Torque);

    UE_LOG(LogTemp, Log, TEXT("Angle Error: %f, %f, %f"), Angle_Error.X, Angle_Error.Y, Angle_Error.Z);
    UE_LOG(LogTemp, Log, TEXT("Angle Velocity Error: %f, %f, %f"), AngleVel_Error.X, AngleVel_Error.Y, AngleVel_Error.Z);
    UE_LOG(LogTemp, Log, TEXT("Angle Velocity: %f, %f, %f"), CurrentVel.X, CurrentVel.Y, CurrentVel.Z);

    UE_LOG(LogTemp, Log, TEXT("U1 U2 U3: %f %f %f"), U_1, U_2, U_3);

    UE_LOG(LogTemp, Log, TEXT("Des Acc: %f, %f, %f"), DesAcc.X, DesAcc.Y, DesAcc.Z);
    //UE_LOG(LogTemp, Log, TEXT("Vel Setpoint: %f %f %f"), VelSetpoint.X, VelSetpoint.Y, VelSetpoint.Z);
    UE_LOG(LogTemp, Log, TEXT("Des Angle: %f, %f, %f"), Des_Angle.X, Des_Angle.Y, Des_Angle.Z);

    UE_LOG(LogTemp, Log, TEXT("P Terms: %f, %f, %f"), PTerm.X, PTerm.Y, PTerm.Z);
    UE_LOG(LogTemp, Log, TEXT("I Terms: %f, %f, %f"), ITerm.X, ITerm.Y, ITerm.Z);
    UE_LOG(LogTemp, Log, TEXT("D Terms: %f, %f, %f"), DTerm.X, DTerm.Y, DTerm.Z);

    UE_LOG(LogTemp, Log, TEXT("P Terms Angle: %f, %f, %f"), AnglePTerm.X, AnglePTerm.Y, AnglePTerm.Z);
    UE_LOG(LogTemp, Log, TEXT("I Terms Angle: %f, %f, %f"), AngleITerm.X, AngleITerm.Y, AngleITerm.Z);
    UE_LOG(LogTemp, Log, TEXT("D Terms Angle: %f, %f, %f"), AngleDTerm.X, AngleDTerm.Y, AngleDTerm.Z);

    UE_LOG(LogTemp, Log, TEXT("Motor RPMs: %f, %f, %f, %f"), Motor_RPMs.X, Motor_RPMs.Y, Motor_RPMs.Z, Motor_RPMs.W);

}