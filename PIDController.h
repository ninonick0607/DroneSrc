#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Containers/Queue.h"
#include <vector>
#include <queue>
#include "PIDController.generated.h"

USTRUCT(BlueprintType)
struct FPIDState
{
    GENERATED_BODY()

    UPROPERTY(Transient) 
    FVector ControllerIntegralTerm;  // Initialized to ensure a valid starting value
    FVector AngleIntegralTerm;
//    FVector LastVelocityError;
//
//    int32 PosControllerCurQueueSize = 0.f;
//    int32 AngleControllerCurQueueSize = 0.f;
//
//    FVector Pos_ControllerSumBuffer = FVector::ZeroVector;
//    FVector Angle_ControllerSumBuffer = FVector::ZeroVector;
//
};

USTRUCT(BlueprintType) 
struct FPIDConfig
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector ProportionalGain = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector IntegralGain = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector DerivativeGain = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector AngleProportionalGain = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector AngleIntegralGain = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector AngleDerivativeGain = FVector(0.f, 0.f, 0.f);
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    float MinimumRPM = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    float MaximumRPM = 10464.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Buffer")
    int PositionBuffer_Size;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Buffer")
    int AngleBuffer_Size;


};

USTRUCT(BlueprintType)
struct FPIDResult
{
    GENERATED_BODY()

    UPROPERTY()
    FVector4 MotorRPMs;

    UPROPERTY()
    FVector AngleError;
};

UCLASS()
class DRONEENVIRONMENT_API UPIDController : public UObject
{
    GENERATED_BODY()

public:
    //UFUNCTION(BlueprintCallable, Category = "PID")
    FPIDResult UpdatePID(const FPIDConfig& Config, FPIDState& State, FVector PositionError, FVector DroneVelocity, FRotator DroneOrientation, FVector CurrentAngVel, FVector DroneInertiaTensor,float Mass,float Grav,float DeltaTime);
    void Log_Messages(float Roll_Torque, float Pitch_Torque, float Yaw_Torque, FVector VelSetpoint, float U_1, float U_2, float U_3, FVector4 Motor_RPMs, FVector Des_Angle, FVector Angle_Error, FVector AngleVel_Error, FVector CurrentVel, FVector DesAcc, FVector AngleITerm, FVector AnglePTerm, FVector AngleDTerm, FVector PTerm, FVector ITerm, FVector DTerm);

    //void EnqueueAndMaintainSize(std::queue<FVector>& Queue, FVector Element, int32& CurrentSize, int32 MaxSize, FVector& SumBuffer);

};