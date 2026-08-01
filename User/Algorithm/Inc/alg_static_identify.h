// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ALG_STATIC_IDENTIFY_H
#define ALG_STATIC_IDENTIFY_H

#include "alg_fsm.h"

/* A static-identification cycle is deliberately limited to nine states so
 * the controller uses the project's reusable Class_FSM implementation. */
enum Enum_Static_Identify_FSM_State : uint8_t
{
    Static_Identify_FSM_DISABLED = 0U,
    Static_Identify_FSM_MOVE_FORWARD_APPROACH,
    Static_Identify_FSM_MOVE_FORWARD_POSE,
    Static_Identify_FSM_SETTLE_FORWARD,
    Static_Identify_FSM_SAMPLE_FORWARD,
    Static_Identify_FSM_MOVE_REVERSE_APPROACH,
    Static_Identify_FSM_MOVE_REVERSE_POSE,
    Static_Identify_FSM_SETTLE_REVERSE,
    Static_Identify_FSM_SAMPLE_REVERSE,
};

static constexpr uint8_t STATIC_IDENTIFY_FSM_STATE_COUNT = 9U;

inline float Static_Identify_Profile_Step(float Current, float Target, float Max_Step)
{
    if (Max_Step <= 0.0f)
    {
        return Current;
    }

    float Delta = Target - Current;
    if (Delta > Max_Step)
    {
        return Current + Max_Step;
    }
    if (Delta < -Max_Step)
    {
        return Current - Max_Step;
    }
    return Target;
}

inline void Static_Identify_Apply_Tuning_Targets(
    float *Command_Target, const volatile float *Tuning_Target,
    uint8_t Joint_Count)
{
    for (uint8_t i = 0U; i < Joint_Count; i++)
    {
        Command_Target[i] = Tuning_Target[i];
    }
}

inline void Static_Identify_Mirror_Command_Targets(
    const float *Command_Target, volatile float *Tuning_Target,
    uint8_t Joint_Count)
{
    for (uint8_t i = 0U; i < Joint_Count; i++)
    {
        Tuning_Target[i] = Command_Target[i];
    }
}

class Class_Static_Identify_FSM : public Class_FSM
{
public:
    void Init()
    {
        Class_FSM::Init(STATIC_IDENTIFY_FSM_STATE_COUNT,
                        Static_Identify_FSM_DISABLED);
        Pose_Index = 0U;
    }

    void Reset()
    {
        Class_FSM::Set_Status(Static_Identify_FSM_DISABLED);
        Pose_Index = 0U;
    }

    void TIM_PeriodElapsedCallback(uint8_t Entry_Enable,
                                   uint8_t Arm_Enable,
                                   uint8_t At_Target,
                                   uint16_t Pose_Count,
                                   uint32_t Settle_ms,
                                   uint32_t Sample_ms)
    {
        if (Entry_Enable == 0U)
        {
            if (Get_Now_Status_Serial() != Static_Identify_FSM_DISABLED ||
                Pose_Index != 0U)
            {
                Reset();
            }
            return;
        }

        if (Arm_Enable == 0U)
        {
            return;
        }

        /* A completed run remains disabled while entry stays asserted. */
        if ((Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED) &&
            (Pose_Index >= Pose_Count))
        {
            return;
        }

        if (Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED)
        {
            Class_FSM::Set_Status(Static_Identify_FSM_MOVE_FORWARD_APPROACH);
            return;
        }

        Class_FSM::Reload_TIM_Status_PeriodElapsedCallback();
        switch (Get_Now_Status_Serial())
        {
            case Static_Identify_FSM_MOVE_FORWARD_APPROACH:
                if (At_Target != 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_FORWARD_POSE);
                }
                break;

            case Static_Identify_FSM_MOVE_FORWARD_POSE:
                if (At_Target != 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_SETTLE_FORWARD);
                }
                break;

            case Static_Identify_FSM_SETTLE_FORWARD:
                if (At_Target == 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_FORWARD_APPROACH);
                }
                else if (Status[Get_Now_Status_Serial()].Time >= Settle_ms)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_SAMPLE_FORWARD);
                }
                break;

            case Static_Identify_FSM_SAMPLE_FORWARD:
                if (At_Target == 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_FORWARD_APPROACH);
                }
                else if (Status[Get_Now_Status_Serial()].Time >= Sample_ms)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_REVERSE_APPROACH);
                }
                break;

            case Static_Identify_FSM_MOVE_REVERSE_APPROACH:
                if (At_Target != 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_REVERSE_POSE);
                }
                break;

            case Static_Identify_FSM_MOVE_REVERSE_POSE:
                if (At_Target != 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_SETTLE_REVERSE);
                }
                break;

            case Static_Identify_FSM_SETTLE_REVERSE:
                if (At_Target == 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_REVERSE_APPROACH);
                }
                else if (Status[Get_Now_Status_Serial()].Time >= Settle_ms)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_SAMPLE_REVERSE);
                }
                break;

            case Static_Identify_FSM_SAMPLE_REVERSE:
                if (At_Target == 0U)
                {
                    Class_FSM::Set_Status(Static_Identify_FSM_MOVE_REVERSE_APPROACH);
                }
                else if (Status[Get_Now_Status_Serial()].Time >= Sample_ms)
                {
                    Pose_Index++;
                    if (Pose_Index >= Pose_Count)
                    {
                        Class_FSM::Set_Status(Static_Identify_FSM_DISABLED);
                    }
                    else
                    {
                        Class_FSM::Set_Status(Static_Identify_FSM_MOVE_FORWARD_APPROACH);
                    }
                }
                break;

            default:
                Class_FSM::Set_Status(Static_Identify_FSM_DISABLED);
                break;
        }
    }

    inline uint16_t Get_Pose_Index() const
    {
        return Pose_Index;
    }

private:
    uint16_t Pose_Index = 0U;
};

#endif
