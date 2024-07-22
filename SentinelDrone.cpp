#include "SentinelDrone.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "PIDController.h" 
#include "Math/UnrealMathUtility.h"
#include "Misc/FileHelper.h"
#include "Algo/Accumulate.h"
#include "Containers/Queue.h"
#include "HAL/PlatformFilemanager.h"
#include "Async/ParallelFor.h"
#include "Async/Async.h"

/*

Units:
Position: m
Velocity: m/s
Angle: Radians
Angular Velocity: rads/sec

Convert everything to this ^
*/



// Sets default values
ASentinelDrone::ASentinelDrone()
{
    // Set this pawn to call Tick() every frame.
    PrimaryActorTick.bCanEverTick = true;

    // Initialize Drone Body and Attach Camera
    DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    RootComponent = DroneBody;

    // Initialize Propellers and Attach to Drone Body
    const FString PropellerNames[] = { TEXT("Propeller1"), TEXT("Propeller2"), TEXT("Propeller3"), TEXT("Propeller4") };
    const FString SocketNames[] = { TEXT("PropellerSocket1"), TEXT("PropellerSocket2"), TEXT("PropellerSocket3"), TEXT("PropellerSocket4") };

    Propellers.SetNum(4);
    for (int i = 0; i < 4; ++i) {
        Propellers[i] = CreateDefaultSubobject<UStaticMeshComponent>(*PropellerNames[i]);
        Propellers[i]->SetupAttachment(DroneBody, *SocketNames[i]);
    }

    // Add Camera component
    DroneCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("DroneCamera"));
    DroneCamera->SetupAttachment(DroneBody);
    DroneCamera->SetRelativeLocationAndRotation(FVector(-50.0f, 0.0f, 50.0f), FRotator(-45.0f, 0.0f, 0.0f));

    // PID Controller Initialization
    PIDController = CreateDefaultSubobject<UPIDController>(TEXT("PIDController"));
    Pos_Setpoint = {0.f,0.f,0.f}; 
    //PIDState.LastVelocityError = FVector::ZeroVector; 

    // Initialize CSV and time tracking
    TotalElapsedTime = 0.0f;
    Tick_num = 0.f;

    // Initialize RPMs
    RPMs.Init(0.0f, 4);
}

void ASentinelDrone::BeginPlay()
{
    Super::BeginPlay();
    //PIDState.ControllerIntegralTerm = {0.f,0.f,0.f};
    //PIDState.AngleIntegralTerm = { 0.f,0.f,0.f };

    // Set the drone's rotation to zero
    SetActorRotation(FRotator::ZeroRotator);

    // Initialize CSV with header
    FString CSVFilePath = FPaths::ProjectDir() + "/DroneData.csv";
    FString CSVHeader = "ElapsedTime,DesiredLocationX,DesiredLocationY,DesiredLocationZ,CurrentLocationX,CurrentLocationY,CurrentLocationZ,POS_ErrorX,POS_ErrorY,POS_ErrorZ,Vel_ErrorX,Vel_ErrorY,Vel_ErrorZ,PIDRPM1,PIDRPM2,PIDRPM3,PIDRPM4,Mass\n";
    FFileHelper::SaveStringToFile(CSVHeader, *CSVFilePath);


}

void ASentinelDrone::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    TotalElapsedTime += DeltaTime;
    Tick_num += 1.f;
    UE_LOG(LogTemp, Log, TEXT("TickNum: %f"), Tick_num);
    ExecutePIDControl(DeltaTime);
    UpdatePropellerRotation(DeltaTime);
}
//
//void ASentinelDrone::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
//{
//    Super::SetupPlayerInputComponent(PlayerInputComponent);
//    PlayerInputComponent->BindAxis("MoveUp", this, &ASentinelDrone::MoveAltitude);
//}

void ASentinelDrone::ExecutePIDControl(float DeltaTime)
{


    FVector MyPosition = GetActorLocation();
    FVector MyVelocity = DroneBody->GetComponentVelocity();
    FRotator CurrentOrientation = GetActorRotation();
    FRotator CurrentOrientationRadians;

    CurrentOrientationRadians.Roll = FMath::DegreesToRadians(CurrentOrientation.Roll);
    CurrentOrientationRadians.Pitch = FMath::DegreesToRadians(CurrentOrientation.Pitch);
    CurrentOrientationRadians.Yaw = FMath::DegreesToRadians(CurrentOrientation.Yaw);
    GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, FString::Printf(TEXT("Rotation: %f %f %f"), CurrentOrientationRadians.Roll, CurrentOrientationRadians.Pitch, CurrentOrientationRadians.Yaw));
 
    FVector CurrentAngularVelocity = DroneBody->GetPhysicsAngularVelocityInRadians();
    FVector InertiaTensor = DroneBody->GetInertiaTensor();
    float DroneMass = DroneBody->GetMass();
    float GravityZ = GetWorld()->GetGravityZ();

    FVector PositionError = Pos_Setpoint - MyPosition;
    FVector VelocityError = -MyVelocity;
    UE_LOG(LogTemp, Log, TEXT("Inertia Setpoint: %f %f %f"), InertiaTensor.X, InertiaTensor.Y, InertiaTensor.Z);

    //Change Begins now
    FPIDResult PIDResult = PIDController->UpdatePID(PIDConfig, PIDState, PositionError, VelocityError, CurrentOrientationRadians, CurrentAngularVelocity, InertiaTensor,DroneMass,GravityZ,DeltaTime);

    RPMs[0] = PIDResult.MotorRPMs.X;
    RPMs[1] = PIDResult.MotorRPMs.Y;
    RPMs[2] = PIDResult.MotorRPMs.Z;
    RPMs[3] = PIDResult.MotorRPMs.W;

    CalculateAllAngularVelocities();
    CalculateAllThrusts();
    ApplyAllThrusts();

    //Log data with the correct velocity values and PID values
    LogDataToCSV(this->TotalElapsedTime, Pos_Setpoint, MyPosition, PositionError,VelocityError, PIDResult.MotorRPMs,DroneMass);
    LogMessages(this->TotalElapsedTime, GravityZ, this->Thrusts, Pos_Setpoint, MyPosition, PositionError, PIDResult.MotorRPMs);
}

void ASentinelDrone::CalculateAllAngularVelocities()
{
    AngularVelocities.Empty();
    AngularVelocities.SetNum(RPMs.Num());
    for (int i = 0; i < RPMs.Num(); ++i)
    {
        AngularVelocities[i] = (RPMs[i] * 2.0f * PI) / 60.0f;
    }
}

// Function to calculate thrust based on angular velocity
float ASentinelDrone::CalculateThrust(float InAngularVelocity)
{
    const float CT = 0.000010546f; // Thrust coefficient
    return 100 * (CT * FMath::Square(InAngularVelocity)); // Thrust is multiplied by 100 to go from cm/s^2 to m/s^2
}

void ASentinelDrone::CalculateAllThrusts()
{
    Thrusts.Empty();
    Thrusts.SetNum(AngularVelocities.Num());
    for (int i = 0; i < AngularVelocities.Num(); ++i)
    {
        Thrusts[i] = CalculateThrust(AngularVelocities[i]);
    }
}

// Function to apply thrust
void ASentinelDrone::ApplyThrust(float Thrust, FName SocketName)
{
    FVector ForceLocation = DroneBody->GetSocketLocation(SocketName);
    FVector ThrustDirection = DroneBody->GetUpVector() * Thrust;
    DroneBody->AddForceAtLocation(ThrustDirection, ForceLocation, NAME_None);
    VisualizeThrustVector(ForceLocation, ThrustDirection);
}

void ASentinelDrone::ApplyAllThrusts()
{
    TArray<FName> Sockets = { TEXT("MotorSocket1"), TEXT("MotorSocket2"), TEXT("MotorSocket3"), TEXT("MotorSocket4") };
    const int32 NumThrusts = Thrusts.Num();
    TArray<TFuture<void>> Tasks;

    for (int32 i = 0; i < NumThrusts; ++i)
    {
        // Launch async tasks for thrust calculation
        Tasks.Add(Async(EAsyncExecution::ThreadPool, [this, i, &Sockets]()
            {
                float Thrust = Thrusts[i];
                FName Socket = Sockets[i];

                // Defer ApplyThrust to the game thread to ensure thread safety
                AsyncTask(ENamedThreads::GameThread, [this, Thrust, Socket]()
                    {
                        ApplyThrust(Thrust, Socket);
                    });
            }));
    }

    // Wait for all tasks to complete
    for (TFuture<void>& Task : Tasks)
    {
        Task.Wait();
    }
}

// Function to update propeller rotation
void ASentinelDrone::UpdatePropellerRotation(float DeltaTime)
{
    // Rotate each propeller
    for (int i = 0; i < Propellers.Num(); ++i)
    {
        float InAngularVelocity = AngularVelocities[i];
        Propellers[i]->AddLocalRotation(FRotator(0.0f, InAngularVelocity * DeltaTime, 0.0f));
    }
}

// Function to set propeller RPM
void ASentinelDrone::SetPropellerRPM(int PropellerIndex, float RPM)
{
    if (PropellerIndex >= 0 && PropellerIndex < RPMs.Num())
    {
        RPMs[PropellerIndex] = RPM;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid Propeller Index"));
    }
}

// Function to visualize the thrust vector
void ASentinelDrone::VisualizeThrustVector(const FVector& StartLocation, const FVector& ThrustVector)
{
    FVector EndLocation = StartLocation + ThrustVector / 10;
    DrawDebugDirectionalArrow(GetWorld(), StartLocation, EndLocation, 10.0f, FColor::Green, false, -1, 0, 1);
}

void ASentinelDrone::LogDataToCSV(float ElapsedTime, FVector PosSetpoint, FVector CurrentLocation, FVector Pos_Error, FVector Vel_Error, FVector4 PIDOutput, float DroneMass)
{
    FString CSVFilePath = FPaths::ProjectDir() + "/DroneData.csv";

    // Ensure that all positions and errors are in the same units (assuming cm here)
    FVector PosSetpointCm = PosSetpoint;
    FVector CurrentLocationCm = CurrentLocation;
    FVector PosErrorCm = Pos_Error;
    FVector VelErrorCm = Vel_Error;

    FString CSVLine = FString::Printf(TEXT("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"),
        ElapsedTime,
        PosSetpointCm.X, PosSetpointCm.Y, PosSetpointCm.Z,
        CurrentLocationCm.X, CurrentLocationCm.Y, CurrentLocationCm.Z,
        PosErrorCm.X, PosErrorCm.Y, PosErrorCm.Z,
        VelErrorCm.X, VelErrorCm.Y, VelErrorCm.Z,
        PIDOutput.X, PIDOutput.Y, PIDOutput.Z, PIDOutput.W,
        DroneMass);

    // Save the CSV line to file
    FFileHelper::SaveStringToFile(CSVLine, *CSVFilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
}

void ASentinelDrone::LogMessages(float TimeElapsed, float WorldGravity, const TArray<float>& Thrusters, FVector PosSetpoint, FVector CurrentPosition, FVector Pos_Error, FVector4 PID_Output)

{
    UE_LOG(LogTemp, Error, TEXT("=== Log Start ==="));

    UE_LOG(LogTemp, Log, TEXT("Time: %f"), TimeElapsed);
    UE_LOG(LogTemp, Log, TEXT("World Gravity: %f"), WorldGravity);

    UE_LOG(LogTemp, Log, TEXT("RPM Values:"));
    UE_LOG(LogTemp, Log, TEXT("  RPM1: %f"), PID_Output.X);
    UE_LOG(LogTemp, Log, TEXT("  RPM2: %f"), PID_Output.Y);
    UE_LOG(LogTemp, Log, TEXT("  RPM3: %f"), PID_Output.Z);
    UE_LOG(LogTemp, Log, TEXT("  RPM4: %f"), PID_Output.W);

    UE_LOG(LogTemp, Log, TEXT("Thrust Values: m/s^2"));
    UE_LOG(LogTemp, Log, TEXT("  Thrust1: %f"), Thrusters[0] / 100);
    UE_LOG(LogTemp, Log, TEXT("  Thrust2: %f"), Thrusters[1] / 100);
    UE_LOG(LogTemp, Log, TEXT("  Thrust3: %f"), Thrusters[2] / 100);
    UE_LOG(LogTemp, Log, TEXT("  Thrust4: %f"), Thrusters[3] / 100);

    UE_LOG(LogTemp, Display, TEXT("Desired Position: %f, %f, %f"), PosSetpoint.X, PosSetpoint.Y, PosSetpoint.Z);
    UE_LOG(LogTemp, Display, TEXT("Current Position: %f, %f, %f"), CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z);

    UE_LOG(LogTemp, Warning, TEXT("Error: %f,%f,%f"), Pos_Error.X, Pos_Error.Y, Pos_Error.Z);
    //UE_LOG(LogTemp, Display, TEXT("PID Values - P: %f, I: %f, D: %f"), PIDValues[0], PIDValues[1], PIDValues[2]);

    UE_LOG(LogTemp, Error, TEXT("=== Log End ==="));

    // Log to the screen
    if (GEngine)
    {
        int32 Key = -1; // Unique key for the message, use -1 to ensure a new message is added
        float DisplayTime = 5.0f; // Time in seconds to display the message on the screen
        FColor TextColor = FColor::Yellow; // Color of the text
        GEngine->AddOnScreenDebugMessage(Key, DisplayTime, TextColor, FString::Printf(TEXT("Time: %f"), TimeElapsed));
        GEngine->AddOnScreenDebugMessage(Key, DisplayTime, TextColor, FString::Printf(TEXT("Desired Position: %f, %f, %f"), PosSetpoint.X, PosSetpoint.Y, PosSetpoint.Z));
        GEngine->AddOnScreenDebugMessage(Key, DisplayTime, TextColor, FString::Printf(TEXT("Current Position: %f,%f,%f"), CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z));
        GEngine->AddOnScreenDebugMessage(Key, DisplayTime, TextColor, FString::Printf(TEXT("Error: %f,%f,%f"), Pos_Error.X, Pos_Error.Y, Pos_Error.Z));
    }
}

//void ASentinelDrone::MoveAltitude(float Value)
//{
//    // Adjust the AltitudeSetpoint based on input value
//    // Assuming Value ranges from -1 to 1, scale it to your desired rate of change
//    float AltitudeChangeRate = 10.0f; // Units per second, adjust as needed
//    PosSetpoint += Value * AltitudeChangeRate * GetWorld()->GetDeltaSeconds();
//    PosSetpoint = FMath::Clamp(PosSetpoint, 0.0f, 1000.0f); // Clamp to a sensible range
//}
