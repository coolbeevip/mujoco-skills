import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime import vector


@pytest.mark.parametrize("value", [[0, 0], [[0, 0, 0]], [0, np.nan, 0], [np.inf, 0, 0]])
def test_invalid_velocity_is_rejected(value):
    with pytest.raises(ValueError, match="velocity"):
        vector(value, 3, "velocity")


def test_valid_velocity_retains_units():
    np.testing.assert_array_equal(
        vector([0.25, -0.125, 0.5], 3, "velocity"), [0.25, -0.125, 0.5]
    )
