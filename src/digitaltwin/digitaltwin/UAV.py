import numpy as np
import scipy as sp
import scipy.interpolate
from itertools import product

from digitaltwin.utils import *

class UAV():
    def __init__(self, config_fpath='./src/digitaltwin/inputfiles/UAVconfig.json'):
        self.config = read_json_file(config_fpath)
        self.measurementGenerator = measurementGenerator(self.config["states"], self.config["controls"], self.config["observations"])

class measurementGenerator():
    def __init__(self,  states, controls, observations, noise = None):
        self.states = states
        self.controls = controls
        self.observations = observations
        self.noise = noise if noise is not None else noiseParams()

    def getMeasurement(self, stateIdx, controlIdx, noisy = True, type = 'linear'):
        # Choose the interpolation type
        if type is 'linear':
            # Create coordinate pairs
            lists = [range(0,len(self.states[0])),range(0,len(self.states[0])), range(0,len(self.controls))]
            coord = list(product(*lists))

            # create data matrix
            data = []
            for state1, state2, control in coord:
                data.append(self.observations[str(self.states[0][state1])][str(self.states[1][state2])][self.controls[control]]["mean"])
            # Create interpolator object
            interp = scipy.interpolate.LinearNDInterpolator(coord, data)
        else:
            print('Error: Unknown interpolation type:'+str(type))

        # Generate clean measurement
        cleanmeasurement = interp(stateIdx[0], stateIdx[1], controlIdx)[0]

        if noisy:
            # Add artificial noise to measurement
            if self.noise.type is "Gaussian":
                noise = np.random.normal(self.noise.mean, self.noise.sigma,cleanmeasurement.shape)
                noisymeasurement = cleanmeasurement+noise
                noisymeasurement = noisymeasurement.clip(min=1)
            else:
                noisymeasurement = cleanmeasurement

        # normalize data by load factor
        if self.controls[controlIdx[0]] == '2g':
            cleanmeasurement = [x/2.0 for x in cleanmeasurement]
            noisymeasurement = [x/2.0 for x in noisymeasurement]
        elif self.controls[controlIdx[0]] == '3g':
            cleanmeasurement = [x/3.0 for x in cleanmeasurement]
            noisymeasurement = [x/3.0 for x in noisymeasurement]

        if noisy:
            return noisymeasurement
        else:
            return cleanmeasurement

class noiseParams():
    def __init__(self, type = "Gaussian", sigma=150):
        self.type = type
        self.mean = 0
        self.sigma = sigma
