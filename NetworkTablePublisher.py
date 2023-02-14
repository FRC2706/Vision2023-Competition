from ntcore import NetworkTableInstance
from ntcore import NetworkTable

def publishNumber(MergeVisionPipeLineTableName, name, value):
    #start NetworkTable
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = ntinst.getTable(MergeVisionPipeLineTableName)
    networkTable.putNumber(name, value)
    #print(name+ ": " + str(value))

def publishString(NetworkTableName,name, Strvalue):
    #start NetworkTable
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = NetworkTable.getTable(NetworkTableName)
    networkTable.putString(name, Strvalue)
    #print(name+ ": " + str(value))    