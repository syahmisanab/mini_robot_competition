import logging
import datetime
import os

def setup_logging(base_log_path):
    """
    Sets up logging to a file with a timestamp in its name and also to console.
    Returns the path to the created log file.
    """
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_file_name = f"{base_log_path}_{timestamp}.txt"
    log_dir = os.path.dirname(log_file_name)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Reset any existing handlers to avoid duplicate log entries if called multiple times
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Configure logging to write to the file and console
    logging.basicConfig(
        level=logging.INFO, # Default logging level. Change to DEBUG for more verbose logs.
        format='%(asctime)s - %(processName)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file_name),
            logging.StreamHandler() # Also print to console
        ]
    )
    # Set logging level for specific modules if needed
    logging.getLogger('ultralytics').setLevel(logging.WARNING) # Reduce YOLO verbose output

    logging.info(f"Logging initialized to {log_file_name}")
    return log_file_name

def constrain(val, min_val, max_val):
    """Clamps a value between a minimum and maximum."""
    return max(min_val, min(val, max_val))