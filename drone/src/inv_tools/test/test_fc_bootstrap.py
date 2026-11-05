from inv_tools.fc_bootstrap import dds_outcome


def test_dds_success_and_failure_messages():
    # Exact STATUSTEXTs from AP_DDS_Client.cpp at Copter-4.7.2.
    assert dds_outcome("DDS: Initialization passed") is True
    assert dds_outcome("DDS: Creation Requests failed") is False
    assert dds_outcome("DDS: No ping response, exiting") is False
    assert dds_outcome("DDS: Transport initialization failed") is False


def test_progress_messages_are_not_outcomes():
    for text in (
        "DDS: initializing...",
        "DDS: Using UDP",
        "DDS: No ping response, retrying",
        "DDS: Participant session request failure",
        "EKF3 IMU0 origin set",
    ):
        assert dds_outcome(text) is None, text
