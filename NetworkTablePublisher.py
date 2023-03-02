from ntcore import NetworkTableInstance

def publishNumber(MergeVisionPipeLineTableName, name, value):
    #start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networktable = ntinst.getTable(MergeVisionPipeLineTableName)
    networktable.putNumber(name, value)
    #print(name+ ": " + str(value))

def publishString(MergeVisionPipeLineTableName,name, Strvalue):
    #start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networktable = ntinst.getTable(MergeVisionPipeLineTableName)
    networktable.putString(name, Strvalue)
    #print(name+ ": " + str(value))    

def publishBoolean(MergeVisionPipeLineTableName,name, Booleanvalue):
    #start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networktable = ntinst.getTable(MergeVisionPipeLineTableName)
    networktable.putBoolean(name, Booleanvalue)
    #print(name+ ": " + str(value))  

def publishNumberArray(MergeVisionPipeLineTableName,name, numarray):
    #start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networktable = ntinst.getTable(MergeVisionPipeLineTableName)
    networktable.putNumberArray(numarray)    