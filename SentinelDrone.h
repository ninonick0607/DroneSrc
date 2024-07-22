#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PIDController.h"
#include "Camera/CameraComponent.h"
#include "Containers/Queue.h"
#include <vector>
#include <queue>
#include "Components/StaticMeshComponent.h"
#include "SentinelDrone.generated.h"

UCLASS()
class DRONEENVIRONMENT_API ASentinelDrone : public APawn
{
    GENERATED_BODY()

public:
    ASentinelDrone();

protected:
    virtual void BeginPlay() override;
    //virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

public:
    virtual void Tick(float DeltaTime) override;

    UFUNCTION(BlueprintCallable, Category = "DroneControl")
    void SetPropellerRPM(int PropellerIndex, float RPM);

    // PID Controller and configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    UPIDController* PIDController;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FPIDConfig PIDConfig;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
    FVector Pos_Setpoint;

private:
    void ExecutePIDControl(float DeltaTime);
    void CalculateAllAngularVelocities();
    void CalculateAllThrusts();
    void ApplyAllThrusts();
    void UpdatePropellerRotation(float DeltaTime);
    void VisualizeThrustVector(const FVector& StartLocation, const FVector& ThrustVector);
    float CalculateThrust(float AngularVelocity);
    void ApplyThrust(float Thrust, FName SocketName);
    void LogDataToCSV(float ElapsedTime, FVector PosSetpoint, FVector CurrentLocation, FVector Pos_Error, FVector Vel_Error, FVector4 PIDOutput, float DroneMass);
    void LogMessages(float TimeElapsed, float WorldGravity, const TArray<float>& Thrusters, FVector PosSetpoint, FVector CurrentPosition, FVector Pos_Error, FVector4 PID_Output);
    //void MoveAltitude(float Value);



    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* DroneBody;

    UPROPERTY(VisibleAnywhere)
    TArray<UStaticMeshComponent*> Propellers;

    UPROPERTY(VisibleAnywhere)
    UCameraComponent* DroneCamera;



    FPIDState PIDState;

    TArray<float> RPMs;
    TArray<float> AngularVelocities;
    TArray<float> Thrusts;

    //float Mass;
    FVector Dimensions;
    FVector Position;
    FVector Velocity;
    FRotator Orientation;
    FVector AngularVelocity;

    float TotalElapsedTime;
    float Tick_num;

//
//private:
//
//    std::queue<FVector> PosControllerBuffer;
//    std::queue<FVector> AngleControllerBuffer;

};
