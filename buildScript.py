import zipfile

FILENAME = "visionTape21.zip"
FILENAME2 = "visionIntake22.zip"
FILENAME3 = "visionAprilTag23.zip"

#create a ZipFile object
zipObj = zipfile.ZipFile(FILENAME, 'w')

# Add module files to the zip
zipObj.write('DistanceFunctions.py')
zipObj.write('FindTape.py')
zipObj.write('FindCone.py')
zipObj.write('FindCube.py')
zipObj.write('DetectIntakeItem.py')
zipObj.write('FindAprilTag.py')
zipObj.write('VisionConstants.py')
zipObj.write('VisionMasking.py')
zipObj.write('VisionUtilities.py')
zipObj.write('NetworkTablePublisher.py')
zipObj.write('DriverOverlay.py')
zipObj.write('MergeFRCPipeline.py','uploaded.py')
zipObj.write('pipelineTape21.json', 'pipelineConfig.json')


zipObj2 = zipfile.ZipFile(FILENAME2, 'w')

zipObj2.write('DistanceFunctions.py')
zipObj2.write('FindTape.py')
zipObj2.write('FindCone.py')
zipObj2.write('FindCube.py')
zipObj2.write('DetectIntakeItem.py')
zipObj2.write('FindAprilTag.py')
zipObj2.write('VisionConstants.py')
zipObj2.write('VisionMasking.py')
zipObj2.write('VisionUtilities.py')
zipObj2.write('NetworkTablePublisher.py')
zipObj2.write('DriverOverlay.py')
z,ipObj2.write('MergeFRCPipeline.py','uploaded.py')
zipObj2.write('pipelineIntake22.json', 'pipelineConfig.json')

zipObj3 = zipfile.ZipFile(FILENAME3, 'w')

zipObj3.write('DistanceFunctions.py')
zipObj3.write('FindTape.py')
zipObj3.write('FindCone.py')
zipObj3.write('FindCube.py')
zipObj3.write('DetectIntakeItem.py')
zipObj3.write('FindAprilTag.py')
zipObj3.write('VisionConstants.py')
zipObj3.write('VisionMasking.py')
zipObj3.write('VisionUtilities.py')
zipObj3.write('NetworkTablePublisher.py')
zipObj3.write('DriverOverlay.py')
zipObj3.write('MergeFRCPipeline.py','uploaded.py')
zipObj3.write('pipelineAprilTag23.json', 'pipelineConfig.json')

print("I have wrote the file: " + FILENAME + ", " + FILENAME2 + ", " + FILENAME3)