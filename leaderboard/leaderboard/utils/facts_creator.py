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
            common_facts["outside_route"] = (criterion.test_status == "FAILURE")

        elif name == "RunningRedLightTest":
            common_facts["running_red_light"] = (criterion.test_status == "FAILURE")

        elif name == "RunningStopTest":
            common_facts["running_stop"] = (criterion.test_status == "FAILURE")

        elif name == "AgentBlockedTest":
            common_facts["agent_blocked"] = (criterion.test_status == "FAILURE")

        elif name == "RouteCompletionTest":
            common_facts["route_completed"] = (criterion.test_status == "SUCCESS")

        elif name == "MinTTCAutoCriterion":
            print(criterion.actual_value)
            common_facts["min_ttc"] = float(criterion.actual_value)

    return common_facts

# missing car_private_facats extracts

# High speed temporary construction_private_facats extracts

# High-speed reckless lane cutting_private_facats extracts

# Highway accident vehicle_private_facats extracts

# Trucks encountered during construction_private_facats extracts

# Drive into the roundabout_private_facats extracts

# Four students crossing the road_private_facats extracts

# avoid a disabled vehicle_private_facats extracts

# Slanted motor and children_private_facats extracts

# reverse vehicle_private_facats extracts
def extract_private_facts_reverse_vehicle(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "ReverseVehicleBrakeCriterion":
            facts["brake_response"] = (criterion.brake_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleBypassCriterion":
            facts["safe_bypass"] = (criterion.bypass_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleResumeCriterion":
            facts["resume_route"] = (criterion.resume_status == "SUCCESS")

    return facts

# crazy motor_private_facats extracts

# Blind spot hidden car_private_facats extracts