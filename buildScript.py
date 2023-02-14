import zipfile

FILENAME = "visionCompPi21.zip"
FILENAME2 = "visionCompPi22.zip"
FILENAME3 = "visionCompPi23.zip"

#create a ZipFile object
zipObj = zipfile.ZipFile(FILENAME, 'w')

# Add module files to the zip
zipObj.write('DistanceFunctions.py')
zipObj.write('FindTarget.py')
zipObj.write('VisionConstants.py')
zipObj.write('VisionMasking.py')
zipObj.write('VisionUtilities.py')
zipObj.write('NetworkTablePublisher.py')
zipObj.write('DriverOverlay.py')
zipObj.write('MergeFRCPipeline.py','uploaded.py')
zipObj.write('pipelineConfigPi20.json', 'pipelineConfig.json')


zipObj2 = zipfile.ZipFile(FILENAME2, 'w')

zipObj2.write('DistanceFunctions.py')
zipObj2.write('FindCone.py')
zipObj2.write('FindCube.py')
zipObj2.write('FindGamePiece.py')
zipObj2.write('FindTarget.py')
zipObj2.write('FindColourRange.py')
zipObj2.write('VisionConstants.py')
zipObj2.write('VisionMasking.py')
zipObj2.write('VisionUtilities.py')
zipObj2.write('NetworkTablePublisher.py')
zipObj2.write('DriverOverlay.py')
zipObj2.write('DetectIntakeItem.py')
zipObj2.write('MergeFRCPipeline.py','uploaded.py')
zipObj2.write('pipelineConfigPi22.json', 'pipelineConfig.json')

zipObj3 = zipfile.ZipFile(FILENAME3, 'w')

zipObj3.write('DistanceFunctions.py')
zipObj3.write('FindTarget.py')
zipObj3.write('VisionConstants.py')
zipObj3.write('VisionMasking.py')
zipObj3.write('VisionUtilities.py')
zipObj3.write('NetworkTablePublisher.py')
zipObj3.write('DriverOverlay.py')
zipObj3.write('MergeFRCPipeline.py','uploaded.py')
zipObj3.write('pipelineConfigPi21.json', 'pipelineConfig.json')

print("I have wrote the file: " + FILENAME + ", " + FILENAME2 + ", " + FILENAME3)