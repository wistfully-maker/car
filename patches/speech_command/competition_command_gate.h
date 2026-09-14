#ifndef COMPETITION_COMMAND_GATE_H
#define COMPETITION_COMMAND_GATE_H

#include <string>

enum class CommandKind
{
    NOISE,
    ORDINARY,
    INCOMPLETE_TASK,
    COMPLETE_TASK,
    DUPLICATE_TASK
};

struct CommandDecision
{
    CommandKind kind;
    std::string text;
};

class CompetitionCommandGate
{
public:
    CompetitionCommandGate() : completed_(false)
    {
    }

    void reset()
    {
        candidate_.clear();
        completed_ = false;
    }

    CommandDecision accept(const std::string &text)
    {
        if (isWakeWordNoise(text))
        {
            return decision(CommandKind::NOISE, text);
        }
        std::string normalized = stripWakeWordPrefix(text);
        if (completed_)
        {
            return decision(CommandKind::DUPLICATE_TASK, candidate_);
        }

        if (
            candidate_.empty() &&
            !startsCompetitionSession(normalized)
        )
        {
            return decision(CommandKind::ORDINARY, normalized);
        }

        mergeCandidate(normalized);
        if (!isComplete(candidate_))
        {
            return decision(CommandKind::INCOMPLETE_TASK, candidate_);
        }

        completed_ = true;
        return decision(CommandKind::COMPLETE_TASK, candidate_);
    }

private:
    std::string candidate_;
    bool completed_;

    static CommandDecision decision(
        CommandKind kind,
        const std::string &text
    )
    {
        CommandDecision result;
        result.kind = kind;
        result.text = text;
        return result;
    }

    static bool contains(const std::string &text, const char *part)
    {
        return text.find(part) != std::string::npos;
    }

    static bool isWakeWordNoise(const std::string &text)
    {
        return text == "飞" ||
            text == "小飞" ||
            text == "小飞小飞";
    }

    static std::string stripWakeWordPrefix(const std::string &text)
    {
        const char *prefixes[] = {"小飞小飞", "小飞", "飞"};
        for (int index = 0; index < 3; ++index)
        {
            std::string prefix(prefixes[index]);
            if (text.compare(0, prefix.size(), prefix) == 0)
            {
                return text.substr(prefix.size());
            }
        }
        return text;
    }

    static int occurrenceCount(
        const std::string &text,
        const char *part
    )
    {
        int count = 0;
        std::string::size_type position = 0;
        std::string needle(part);
        while (
            (position = text.find(needle, position)) !=
            std::string::npos
        )
        {
            ++count;
            position += needle.size();
        }
        return count;
    }

    static int categoryCount(const std::string &text)
    {
        const char *categories[] = {"食品", "日用品", "电子产品"};
        int count = 0;
        for (int index = 0; index < 3; ++index)
        {
            if (contains(text, categories[index]))
            {
                ++count;
            }
        }
        return count;
    }

    static bool startsCompetitionSession(const std::string &text)
    {
        return contains(text, "领取区") ||
            contains(text, "仿真环境");
    }

    static bool isComplete(const std::string &text)
    {
        return contains(text, "领取区") &&
            contains(text, "仿真环境") &&
            categoryCount(text) == 2 &&
            occurrenceCount(text, "对应仓库") >= 2;
    }

    void mergeCandidate(const std::string &text)
    {
        if (candidate_.empty())
        {
            candidate_ = text;
            return;
        }
        if (
            contains(text, "领取区") &&
            categoryCount(text) > 0
        )
        {
            candidate_ = text;
            return;
        }
        if (text.find(candidate_) != std::string::npos)
        {
            candidate_ = text;
            return;
        }
        if (candidate_.find(text) != std::string::npos)
        {
            if (
                text == "放置在对应仓库" &&
                occurrenceCount(candidate_, "对应仓库") < 2
            )
            {
                candidate_ += text;
            }
            return;
        }

        std::string::size_type max_overlap =
            candidate_.size() < text.size() ?
            candidate_.size() : text.size();
        for (std::string::size_type overlap = max_overlap;
             overlap > 0;
             --overlap)
        {
            if (
                candidate_.compare(
                    candidate_.size() - overlap,
                    overlap,
                    text,
                    0,
                    overlap
                ) == 0
            )
            {
                candidate_ += text.substr(overlap);
                return;
            }
        }
        candidate_ += text;
    }
};

#endif
