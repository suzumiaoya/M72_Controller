#include <cmath>
#include <cstdlib>
#include <iostream>

#include "alg_static_identify.h"

namespace
{
int Failure_Count = 0;

void Expect(bool Condition, const char *Message)
{
    if (!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        Failure_Count++;
    }
}

void Tick(Class_Static_Identify_FSM &FSM, uint8_t At_Target,
          uint16_t Pose_Count = 2U, uint32_t Settle_ms = 3U,
          uint32_t Sample_ms = 3U)
{
    FSM.TIM_PeriodElapsedCallback(1U, 1U, At_Target, Pose_Count,
                                  Settle_ms, Sample_ms);
}

void Test_State_Order_And_Timing()
{
    Class_Static_Identify_FSM fsm;
    fsm.Init();
    Expect(fsm.Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED,
           "FSM starts disabled");

    Tick(fsm, 0U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_FORWARD_APPROACH,
           "entry starts forward approach");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_FORWARD_POSE,
           "forward approach precedes nominal pose");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SETTLE_FORWARD,
           "forward pose precedes settle");

    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SETTLE_FORWARD,
           "forward settle lasts the configured interval");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SAMPLE_FORWARD,
           "forward settle transitions to sample");

    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SAMPLE_FORWARD,
           "forward sample lasts the configured interval");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_REVERSE_APPROACH,
           "forward sample precedes reverse approach");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_REVERSE_POSE,
           "reverse approach precedes nominal pose");
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SETTLE_REVERSE,
           "reverse pose precedes settle");

    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_SAMPLE_REVERSE,
           "reverse settle transitions to sample");
    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Tick(fsm, 1U);
    Expect(fsm.Get_Pose_Index() == 1U,
           "reverse sample advances the pose index");
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_FORWARD_APPROACH,
           "next pose restarts with forward approach");
}

void Test_Tolerance_Retry_Direction()
{
    const uint8_t forward_states[] = {
        Static_Identify_FSM_SETTLE_FORWARD,
        Static_Identify_FSM_SAMPLE_FORWARD,
    };
    for (uint8_t state : forward_states)
    {
        Class_Static_Identify_FSM fsm;
        fsm.Init();
        fsm.Set_Status(state);
        Tick(fsm, 0U);
        Expect(fsm.Get_Now_Status_Serial() ==
                   Static_Identify_FSM_MOVE_FORWARD_APPROACH,
               "forward tolerance loss repeats forward approach");
    }

    const uint8_t reverse_states[] = {
        Static_Identify_FSM_SETTLE_REVERSE,
        Static_Identify_FSM_SAMPLE_REVERSE,
    };
    for (uint8_t state : reverse_states)
    {
        Class_Static_Identify_FSM fsm;
        fsm.Init();
        fsm.Set_Status(state);
        Tick(fsm, 0U);
        Expect(fsm.Get_Now_Status_Serial() ==
                   Static_Identify_FSM_MOVE_REVERSE_APPROACH,
               "reverse tolerance loss repeats reverse approach");
    }
}

void Test_No_Timeout_And_Completion_Hold()
{
    Class_Static_Identify_FSM fsm;
    fsm.Init();
    Tick(fsm, 0U, 1U);
    for (uint32_t i = 0U; i < 100000U; i++)
    {
        Tick(fsm, 0U, 1U);
    }
    Expect(fsm.Get_Now_Status_Serial() ==
               Static_Identify_FSM_MOVE_FORWARD_APPROACH,
           "unreachable target stays in movement without timeout");

    fsm.Init();
    Tick(fsm, 0U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Tick(fsm, 1U, 1U, 1U, 1U);
    Expect(fsm.Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED,
           "last reverse sample finishes in disabled hold state");
    Expect(fsm.Get_Pose_Index() == 1U,
           "completion marker retains pose count");
    fsm.TIM_PeriodElapsedCallback(1U, 0U, 0U, 1U, 1U, 1U);
    Expect(fsm.Get_Pose_Index() == 1U,
           "arm disable preserves the completed run marker");
    Tick(fsm, 1U, 1U, 1U, 1U);
    Expect(fsm.Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED,
           "asserted entry holds completed run");

    fsm.TIM_PeriodElapsedCallback(0U, 1U, 0U, 1U, 1U, 1U);
    Expect(fsm.Get_Pose_Index() == 0U,
           "entry low rearms the pose sequence");
}

void Test_Arm_Disable_Pauses_FSM()
{
    Class_Static_Identify_FSM fsm;
    fsm.Init();
    Tick(fsm, 0U);
    uint8_t state_before = fsm.Get_Now_Status_Serial();
    uint32_t time_before = fsm.Status[state_before].Time;

    fsm.TIM_PeriodElapsedCallback(1U, 0U, 0U, 2U, 3U, 3U);
    Expect(fsm.Get_Now_Status_Serial() == state_before,
           "arm disable pauses instead of resetting the FSM");
    Expect(fsm.Status[state_before].Time == time_before,
           "arm disable pauses the state timer");
}

void Test_Profile_Mirror_And_Direct_Targets()
{
    float command[6] = {0.0f, 0.5f, -0.5f, 1.1f, -1.2f, 0.7f};
    volatile float tuning[3] = {1.0f, -0.5f, 0.25f};

    Static_Identify_Apply_Tuning_Targets(command, tuning, 3U);
    Expect(command[0] == 1.0f && command[1] == -0.5f &&
               command[2] == 0.25f,
           "tuning targets pass through without a profile");
    Expect(command[3] == 1.1f && command[4] == -1.2f &&
               command[5] == 0.7f,
           "direct tuning leaves J3-J5 at their last absolute target");

    const float stage_target[3] = {-0.6f, 0.8f, 0.9f};
    const float max_step = 0.0003f;
    for (uint32_t cycle = 0U; cycle < 1000U; cycle++)
    {
        for (uint8_t joint = 0U; joint < 3U; joint++)
        {
            float previous = command[joint];
            command[joint] = Static_Identify_Profile_Step(
                command[joint], stage_target[joint], max_step);
            Expect(std::fabs(command[joint] - previous) <=
                       max_step + 1.0e-7f,
                   "profile increment respects the per-cycle limit");
        }
        Static_Identify_Mirror_Command_Targets(command, tuning, 3U);
        for (uint8_t joint = 0U; joint < 3U; joint++)
        {
            Expect(tuning[joint] == command[joint],
                   "running profile mirrors every command target");
        }
    }
}

void Test_Exit_Continuity_From_Every_State()
{
    for (uint8_t state = Static_Identify_FSM_DISABLED;
         state < STATIC_IDENTIFY_FSM_STATE_COUNT; state++)
    {
        Class_Static_Identify_FSM fsm;
        fsm.Init();
        fsm.Set_Status(state);

        float command[6] = {
            0.1f * static_cast<float>(state + 1U),
            -0.2f * static_cast<float>(state + 1U),
            0.05f * static_cast<float>(state + 1U),
            0.7f, -0.4f, 1.2f,
        };
        volatile float tuning[3] = {0.0f, 0.0f, 0.0f};
        Static_Identify_Mirror_Command_Targets(command, tuning, 3U);

        float before[6];
        for (uint8_t joint = 0U; joint < 6U; joint++)
        {
            before[joint] = command[joint];
        }

        fsm.TIM_PeriodElapsedCallback(0U, 1U, 0U, 2U, 3U, 3U);
        Static_Identify_Apply_Tuning_Targets(command, tuning, 3U);

        for (uint8_t joint = 0U; joint < 6U; joint++)
        {
            Expect(command[joint] == before[joint],
                   "exiting any state produces no target step");
        }
        Expect(fsm.Get_Now_Status_Serial() == Static_Identify_FSM_DISABLED,
               "entry low disables the FSM from every state");
    }
}
}

int main()
{
    Test_State_Order_And_Timing();
    Test_Tolerance_Retry_Direction();
    Test_No_Timeout_And_Completion_Hold();
    Test_Arm_Disable_Pauses_FSM();
    Test_Profile_Mirror_And_Direct_Targets();
    Test_Exit_Continuity_From_Every_State();

    if (Failure_Count != 0)
    {
        std::cerr << Failure_Count << " test assertion(s) failed\n";
        return EXIT_FAILURE;
    }

    std::cout << "static_identify_fsm_test: PASS\n";
    return EXIT_SUCCESS;
}
