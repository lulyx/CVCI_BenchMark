def extract_common_facts(criteria_list):
    common_facts = {
        "collision": False,
        "min_ttc": None,
        "outside_route": False,
        "running_red_light": False,
        "running_stop": False,
        "agent_blocked": False,
        "route_completed": False,
    }
    for criterion in criteria_list:
        name = criterion.name
        if name == "CollisionTest":
            common_facts["collision"] = (criterion.test_status == "FAILURE" or len(criterion.events) > 0)

        elif name == "OutsideRouteLanesTest":
            # 这里只是示意，具体要看你怎么定义“outside_route”
            common_facts["outside_route"] = (criterion.test_status == "FAILURE")

        elif name == "RunningRedLightTest":
            common_facts["running_red_light"] = (criterion.test_status == "FAILURE")

        elif name == "RunningStopTest":
            common_facts["running_stop"] = (criterion.test_status == "FAILURE")

        elif name == "AgentBlockedTest":
            common_facts["agent_blocked"] = (criterion.test_status == "FAILURE")

        elif name == "RouteCompletionTest":
            common_facts["route_completed"] = (criterion.test_status == "SUCCESS")

        elif name == "MinTTCriterion":
            # 你后面如果自己写一个最小TTC criterion，就可以从 actual_value 里拿
            common_facts["min_ttc"] = criterion.actual_value

    return common_facts


def extract_private_facts_reverse_vehicle(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "ReverseVehicleBrakeCriterion":
            facts["brake_response"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleBypassCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts

def extract_private_facts_roundabout_merge_conflict(criteria_list):
    """
    大转盘极端交互场景的私有事实提取
    """
    facts = {
        "decelerate_response": False,
        "safe_merge": False,
        "yield_convoy": False,
    }

    for criterion in criteria_list:
        if criterion.name == "RoundaboutDecelerateCriterion":
            facts["decelerate_response"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "RoundaboutSafeMergeCriterion":
            facts["safe_merge"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "RoundaboutYieldConvoyCriterion":
            facts["yield_convoy"] = (criterion.test_status == "SUCCESS")

    return facts


def extract_private_facts_broken_down_vehicle(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "BrakeCriterion":
            facts["brake_response"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "BypassCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "ResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts
