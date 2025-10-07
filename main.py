from intrinsic_calibration import IntrinsicCameraCalibrationExtractor
from extrinsic_calibration import ExtrinsicCameraCalibrationExtractor


if __name__ == "__main__":
    extractor = IntrinsicCameraCalibrationExtractor()
    extractor.run()
    extractor = ExtrinsicCameraCalibrationExtractor()
    extractor.run()
    print("Calibration completed.")
