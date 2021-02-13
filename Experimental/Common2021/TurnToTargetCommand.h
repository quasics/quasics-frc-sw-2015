#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <cmath>
#include <iostream>

#include "CommonDriveSubsystem.h"

#include "Constants.h"

class TurnToTargetCommand
    : public frc2::CommandHelper<frc2::CommandBase, TurnToTargetCommand>
{
public:
    TurnToTargetCommand(CommonDriveSubsystem *driveBase, double maxSpeed, double tolerance = 0.05)
        : m_driveBase(driveBase), m_maxSpeed(maxSpeed), m_tolerance(tolerance)
    {
        AddRequirements({m_driveBase});
        auto inst = nt::NetworkTableInstance::GetDefault();
        auto table = inst.GetTable(NetworkTableNames::kVisionTable);
        m_targetXEntry = table->GetEntry(NetworkTableNames::kTargetXEntry);
    }

    void Execute() override
    {
        // Decide which direction we should turn in
        double offset = 0;
        enum Direction
        {
            right,
            left,
            none
        };
        Direction turnDir = none;
        if (!GetOffsetFromCenter(offset))
        {
            turnDir = right;
        }
        else if (CloseEnough(offset))
        {
            turnDir = none;
        }
        else if (offset > 0)
        {
            turnDir = right;
        }
        else if (offset < 0)
        {
            turnDir = left;
        }

        // Execute on the above decision
        switch (turnDir)
        {
        case right:
            m_driveBase->ArcadeDrive(0, m_maxSpeed);
            break;
        case left:
            m_driveBase->ArcadeDrive(0, -m_maxSpeed);
            break;
        case none:
            m_driveBase->Stop();
            break;
        }
    }

    bool IsFinished() override
    {
        double offset = 0;
        if (!GetOffsetFromCenter(offset))
        {
            std::cout << "Can't get offset" << std::endl;
            return false;
        }
        std::cout << "Target offset: " << offset << std::endl;
        return CloseEnough(offset);
    }

    void End(bool interrupted) override
    {
        m_driveBase->Stop();
    }

private:
    bool CloseEnough(double offset)
    {
        return std::abs(offset) <= m_tolerance;
    }
    bool GetOffsetFromCenter(double &offset)
    {
        std::vector<double> arr = m_targetXEntry.GetDoubleArray(std::vector<double>());
        if (arr.empty())
        {
            return false;
        }
        offset = arr[0];
        return true;
    }

private:
    CommonDriveSubsystem *const m_driveBase;
    const double m_maxSpeed;
    const double m_tolerance;
    nt::NetworkTableEntry m_targetXEntry;
};
