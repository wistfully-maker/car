#include "competition_command_gate.h"

#include <cassert>
#include <iostream>
#include <string>

int main()
{
    CompetitionCommandGate gate;

    assert(gate.accept("飞").kind == CommandKind::NOISE);
    assert(gate.accept("小飞").kind == CommandKind::NOISE);
    assert(gate.accept("小飞小飞").kind == CommandKind::NOISE);
    assert(gate.accept("食品").kind == CommandKind::ORDINARY);
    assert(
        gate.accept("请前往物品领取区").kind ==
        CommandKind::INCOMPLETE_TASK
    );
    assert(
        gate.accept("放置在对应仓库").kind ==
        CommandKind::INCOMPLETE_TASK
    );
    assert(gate.accept("取得食品").kind == CommandKind::INCOMPLETE_TASK);

    CommandDecision complete =
        gate.accept("并在仿真环境中取得日用品放置在对应仓库");
    assert(complete.kind == CommandKind::COMPLETE_TASK);
    assert(complete.text.find("食品") != std::string::npos);
    assert(complete.text.find("日用品") != std::string::npos);

    assert(gate.accept(complete.text).kind == CommandKind::DUPLICATE_TASK);

    gate.reset();
    assert(
        gate.accept(
            "请前往物品领取区取得电子产品放置在对应仓库"
            "并在仿真环境中取得食品放置在对应仓库"
        ).kind == CommandKind::COMPLETE_TASK
    );

    gate.reset();
    assert(
        gate.accept("请前往物品领取区").kind ==
        CommandKind::INCOMPLETE_TASK
    );
    assert(
        gate.accept("取得食品和日用品").kind ==
        CommandKind::INCOMPLETE_TASK
    );
    assert(
        gate.accept("仿真环境").kind == CommandKind::INCOMPLETE_TASK
    );
    assert(
        gate.accept("放置在对应仓库放置在对应仓库").kind ==
        CommandKind::COMPLETE_TASK
    );

    gate.reset();
    assert(
        gate.accept("请前往物品领取区取得食品").kind ==
        CommandKind::INCOMPLETE_TASK
    );
    CommandDecision overlap = gate.accept(
        "食品放置在对应仓库并在仿真环境中取得日用品"
        "放置在对应仓库"
    );
    assert(overlap.kind == CommandKind::COMPLETE_TASK);
    assert(overlap.text.find("食品食品") == std::string::npos);
    assert(
        overlap.text.find("食品") ==
        overlap.text.rfind("食品")
    );

    gate.reset();
    CommandDecision wake_prefix = gate.accept(
        "飞请前往物品领取区取得食品放置在对应仓库"
        "并在仿真环境中取得日用品放置在对应仓库"
    );
    assert(wake_prefix.kind == CommandKind::COMPLETE_TASK);
    assert(wake_prefix.text.find("飞") == std::string::npos);
    assert(wake_prefix.text.find("请前往") == 0);

    gate.reset();
    assert(
        gate.accept(
            "请前往物品领取区取得电子产品放置在对应仓库"
            "并在仿真环境中取得食品"
        ).kind == CommandKind::INCOMPLETE_TASK
    );
    assert(
        gate.accept("放置在对应仓库").kind ==
        CommandKind::COMPLETE_TASK
    );

    gate.reset();
    assert(
        gate.accept(
            "取得电子产品放置在对应仓库并在仿真环境中取得食品"
            "放置在对应仓库"
        ).kind == CommandKind::INCOMPLETE_TASK
    );
    CommandDecision corrected = gate.accept(
        "请前往物品领取区取得日用品放置在对应仓库"
        "并在仿真环境中取得电子产品放置在对应仓库"
    );
    assert(corrected.kind == CommandKind::COMPLETE_TASK);
    assert(corrected.text.find("食品") == std::string::npos);
    assert(corrected.text.find("日用品") != std::string::npos);

    std::cout << "competition command gate tests passed" << std::endl;
    return 0;
}
