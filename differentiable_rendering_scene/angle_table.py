import numpy as np

class AngleTable():
    def __init__(self):
        self.ac = np.deg2rad(np.array([
            -13.749,
            -13.5347,
            -13.3174,
            -13.0975,
            -12.8758,
            -12.6498,
            -12.4197,
            -12.1855,
            -11.9476,
            -11.7069,
            -11.4636,
            -11.2173,
            -10.9676,
            -10.7144,
            -10.4579,
            -10.1985,
            -9.9352,
            -9.668,
            -9.3961,
            -9.1195,
            -8.8385,
            -8.5535,
            -8.2654,
            -7.9738,
            -7.6781,
            -7.3793,
            -7.0774,
            -6.7727,
            -6.4652,
            -6.1549,
            -5.8416,
            -5.5246,
            -5.2046,
            -4.8824,
            -4.5578,
            -4.231,
            -3.9019,
            -3.5711,
            -3.2387,
            -2.9044,
            -2.5686,
            -2.2314,
            -1.8925,
            -1.5519,
            -1.2098,
            -0.8664,
            -0.5216,
            -0.1757,
            0.1706,
            0.5169,
            0.8628,
            1.2076,
            1.551,
            1.8931,
            2.2337,
            2.5726,
            2.9098,
            3.2456,
            3.5799,
            3.9123,
            4.2431,
            4.5722,
            4.899,
            5.2236,
            5.5458,
            5.8658,
            6.1828,
            6.4961,
            6.8064,
            7.1139,
            7.4186,
            7.7205,
            8.0193,
            8.315,
            8.6066,
            8.8947,
            9.1797,
            9.4607,
            9.7373,
            10.0092,
            10.2764,
            10.5397,
            10.7991,
            11.0556,
            11.3088,
            11.5585,
            11.8048,
            12.0481,
            12.2888,
            12.5267,
            12.7609,
            12.991,
            13.217,
            13.4387,
            13.6586,
            13.8759,
        ]))
        self.al = np.deg2rad(np.array([
            -13.8562,
            -13.6419,
            -13.4261,
            -13.2075,
            -12.9867,
            -12.7628,
            -12.5348,
            -12.3026,
            -12.0666,
            -11.8273,
            -11.5853,
            -11.3405,
            -11.0925,
            -10.841,
            -10.5862,
            -10.3282,
            -10.0669,
            -9.8016,
            -9.5321,
            -9.2578,
            -8.979,
            -8.696,
            -8.4095,
            -8.1196,
            -7.826,
            -7.5287,
            -7.2284,
            -6.9251,
            -6.619,
            -6.3101,
            -5.9983,
            -5.6831,
            -5.3646,
            -5.0435,
            -4.7201,
            -4.3944,
            -4.0665,
            -3.7365,
            -3.4049,
            -3.0716,
            -2.7365,
            -2.4,
            -2.062,
            -1.7222,
            -1.3809,
            -1.0381,
            -0.694,
            -0.3487,
            -0.0026,
            0.3438,
            0.6899,
            1.0352,
            1.3793,
            1.7221,
            2.0634,
            2.4032,
            2.7412,
            3.0777,
            3.4128,
            3.7461,
            4.0777,
            4.4077,
            4.7356,
            5.0613,
            5.3847,
            5.7058,
            6.0243,
            6.3395,
            6.6513,
            6.9602,
            7.2663,
            7.5696,
            7.8699,
            8.1672,
            8.4608,
            8.7507,
            9.0372,
            9.3202,
            9.599,
            9.8733,
            10.1428,
            10.4081,
            10.6694,
            10.9274,
            11.1822,
            11.4337,
            11.6817,
            11.9265,
            12.1685,
            12.4078,
            12.6438,
            12.876,
            13.104,
            13.3279,
            13.5487,
            13.7673,
        ]))
        self.ar = np.deg2rad(np.array([
                -13.6419,
                -13.4261,
                -13.2075,
                -12.9867,
                -12.7628,
                -12.5348,
                -12.3026,
                -12.0666,
                -11.8273,
                -11.5853,
                -11.3405,
                -11.0925,
                -10.841,
                -10.5862,
                -10.3282,
                -10.0669,
                -9.8016,
                -9.5321,
                -9.2578,
                -8.979,
                -8.696,
                -8.4095,
                -8.1196,
                -7.826,
                -7.5287,
                -7.2284,
                -6.9251,
                -6.619,
                -6.3101,
                -5.9983,
                -5.6831,
                -5.3646,
                -5.0435,
                -4.7201,
                -4.3944,
                -4.0665,
                -3.7365,
                -3.4049,
                -3.0716,
                -2.7365,
                -2.4,
                -2.062,
                -1.7222,
                -1.3809,
                -1.0381,
                -0.694,
                -0.3487,
                -0.0026,
                0.3438,
                0.6899,
                1.0352,
                1.3793,
                1.7221,
                2.0634,
                2.4032,
                2.7412,
                3.0777,
                3.4128,
                3.7461,
                4.0777,
                4.4077,
                4.7356,
                5.0613,
                5.3847,
                5.7058,
                6.0243,
                6.3395,
                6.6513,
                6.9602,
                7.2663,
                7.5696,
                7.8699,
                8.1672,
                8.4608,
                8.7507,
                9.0372,
                9.3202,
                9.599,
                9.8733,
                10.1428,
                10.4081,
                10.6694,
                10.9274,
                11.1822,
                11.4337,
                11.6817,
                11.9265,
                12.1685,
                12.4078,
                12.6438,
                12.876,
                13.104,
                13.3279,
                13.5487,
                13.7673,
                13.9831
            ]))

angle_table = AngleTable()