import numpy as np
import matplotlib as plt


model=np.array()
SensorModel=np.array() #how sensor data reflects current position
SensorNoise=np.array() #take standard deviation from manufacturer
modelChanges=np.array() # how new inputs to model affect position
commandInputs=np.array() # changes that have been made to object

kalmanEstimate=0; #select suitable initial value
projectedError=0 #select value to start off with
errorCovariance=np.array()#distribuition of noise. Take it from manufacturer
kalmanGain=0
def projectStateAhead(Model,prevSate,mChanges,cInputs):
    predictedState=(Model*prevSate)+(mChanges*cInputs)
    return predictedState

def projectErrorCovariance(Model,prevError,errorCovariance):
    modelTranspose=np.transpose(model)
    projectedError=(Model*prevError*modelTranspose)+errorCovariance
    return projectedError

def computeKalmanGain(predictedError,sensorModel,sensorNoise):
    sModelTranspose=np.transpose(sensorModel)
    kalmanGain=predictedError*sModelTranspose*((sensorModel*predictedError*sModelTranspose)+sensorNoise)
    kalmanGain=1/kalmanGain
    return kalmanGain

def estimateUpdatefromSensorMeasurements(preditedState,kalmanGain,sensorMeasurements,sModel):
    kalmanEstimate=preditedState+(kalmanGain*(sensorMeasurements-(sModel*preditedState)))
    return kalmanEstimate

def updateErrorCovariance(kalmanGain,sModel,prevNoiseEstimate):
    n=3 #number of rows and columns for identity matrix
    identity=np.identity()
    Covariance=(identity-(kalmanGain*sModel))*prevNoiseEstimate
    return Covariance



while(True):
    #Filter loop to read sensor measerements, predict and update
    sensorMeasurements=0; # input readings from sensor
    predictedState=projectStateAhead(model,kalmanEstimate,0,0)
    projectedError=projectErrorCovariance(model,projectedError,errorCovariance)
    kalmanGain=computeKalmanGain(projectedError,SensorModel,SensorNoise)
    kalmanEstimate=estimateUpdatefromSensorMeasurements(predictedState,kalmanGain,sensorMeasurements,SensorModel)
    updateErrorCovariance(kalmanGain,SensorModel,projectedError)



