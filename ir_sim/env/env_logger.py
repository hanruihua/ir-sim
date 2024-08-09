from loguru import logger
import sys


class EnvLogger:
    def __init__(self, log_file='ir_sim_error.log', log_level='WARNING'):

        logger.remove()
        logger.add(sys.stdout, level=log_level, format="<green>{time:YYYY-MM-DD HH:mm}</green> | " "<level>{level: <8}</level> | " "<level>{message}</level>")

        if log_file is not None:
            logger.add(log_file, level=log_level)


    def info(self, msg):
        logger.info(msg)

    def error(self, msg):
        logger.error(msg)

    def debug(self, msg):
        logger.debug(msg)

    def warning(self, msg):
        logger.warning(msg)

    # def close(self):
    #     logger.remove(self.log_file)
    #     logger.remove(None)

    # def __del__(self):
    #     self.close()


