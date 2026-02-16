import os
import pytest

def test_mbed_app_json_exists():
    """Verify mbed_app.json exists (critical for mbed OS)."""
    assert os.path.exists("mbed_app.json")

def test_cmake_exists():
    """Verify CMakeLists.txt exists."""
    assert os.path.exists("CMakeLists.txt")
