import logging
from logging import handlers
import os

class Logger(object):
    level_relations = {
        'debug':logging.DEBUG,
        'info':logging.INFO,
        'warning':logging.WARNING,
        'error':logging.ERROR,
        'crit':logging.CRITICAL
    }

    def __init__(self, filename, level='info', when='D', backCount=3, fmt='%(asctime)s - %(levelname)s: %(message)s - %(pathname)s[line:%(lineno)d]'):
        full_file = os.path.dirname(__file__) + '/' + filename
        self.logger = logging.getLogger(full_file)
        format_str = logging.Formatter(fmt) # log format
        self.logger.setLevel(self.level_relations.get(level)) # log level
        sh = logging.StreamHandler() # print to screen
        sh.setFormatter(format_str) # display format
        th = handlers.TimedRotatingFileHandler(filename=full_file,when=when,backupCount=backCount,encoding='utf-8') # 往文件里写入#指定间隔时间自动生成文件的处理器
        #实例化TimedRotatingFileHandler
        #interval是时间间隔，backupCount是备份文件的个数，如果超过这个个数，就会自动删除，when是间隔的时间单位，单位有以下几种：
        # S 秒
        # M 分
        # H 小时
        # D 天
        # W 每星期（interval==0时代表星期一）
        # midnight 每天凌晨
        th.setFormatter(format_str)#设置文件里写入的格式

        if (self.logger.hasHandlers()):
            self.logger.handlers.clear()

        self.logger.addHandler(sh) #把对象加到logger里
        self.logger.addHandler(th)

if __name__ == '__main__':
    log = Logger('robot.log',level='info')
    log.logger.debug('debug')
    log.logger.info('info')
    log.logger.warning('warning')
    log.logger.error('error')
    log.logger.critical('critical')