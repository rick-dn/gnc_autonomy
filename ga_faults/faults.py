"""
Fault definitions - SIM_ parameters to set at injection. The runner reads the
original values first and restores them when the fault is cleared.

Units follow the SITL parameters: GPS glitch in degrees of lat/lon (added
straight to the reported position), accel bias m/s^2, gyro bias rad/s,
compass offset mGauss.
"""

# Metres -> degrees of latitude, close enough for a glitch size
M_PER_DEG_LAT = 111_320.0

FAULTS = {
    'none': {
        'description': 'baseline, no fault',
        'set': {},
    },
    'gps_loss': {
        'description': 'GPS receiver stops',
        'set': {'SIM_GPS1_ENABLE': 0},
    },
    'gps_glitch_5m': {
        'description': 'GPS position jumps 5 m north, fix stays good',
        'set': {'SIM_GPS1_GLTCH_X': 5.0 / M_PER_DEG_LAT},
    },
    'accel_bias': {
        'description': 'IMU 1 accelerometer X bias 0.5 m/s^2',
        'set': {'SIM_ACC1_BIAS_X': 0.5},
    },
    'gyro_bias': {
        'description': 'IMU 1 gyro X bias 0.02 rad/s',
        'set': {'SIM_GYR1_BIAS_X': 0.02},
    },
    'accel_bias_large': {
        'description': 'IMU 1 accelerometer X bias 2.0 m/s^2',
        'set': {'SIM_ACC1_BIAS_X': 2.0},
    },
    'gyro_bias_large': {
        'description': 'IMU 1 gyro X bias 0.1 rad/s',
        'set': {'SIM_GYR1_BIAS_X': 0.1},
    },
    'mag_offset': {
        'description': 'compass 1 X offset set to 200 mGauss',
        'set': {'SIM_MAG1_OFS_X': 200.0},
    },
    'mag_fail': {
        'description': 'compass 1 stops',
        'set': {'SIM_MAG1_FAIL': 1},
    },
}
