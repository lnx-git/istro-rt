import socket
import sys
import time
import cv2
import numpy as np
import logging

HOST = ''
PORT = 7001

VISIONN_MOCK = False  # True    # for testing purposes only: do not use tensorflow

VISIONN_RECV_LEN  = 2048
VISIONN_RECV2_LEN  = 102400
VISIONN_CMD_LEN = 17
VISIONN_CMD_HELLO = '{"VISIONN_HELLO":'
VISIONN_CMD_IMPRQ = '{"VISIONN_IMPRQ":'

logger = None
#logging._warn_preinit_stderr = 0

### logger ###

logger_stdout = None
logger_stderr = None

# https://stackoverflow.com/questions/19425736/how-to-redirect-stdout-and-stderr-to-logger-in-python
class LoggerWriter:
    def __init__(self, level):
        # self.level is really like using log.debug(message)
        # at least in my case
        self.level = level

    def write(self, message):
        # if statement reduces the amount of newlines that are
        # printed to the logger
        if message != '\n':
            self.level(message)

    def flush(self):
        # create a flush method so things can be flushed when
        # the system wants to. Not sure if simply 'printing'
        # sys.stderr is the correct way to do it, but it seemed
        # to work properly for me.
        self.level(sys.stderr)

def logger_init(name, filename):
    log = logging.getLogger(name)
    log.setLevel(logging.DEBUG)

    handler = logging.FileHandler(filename, mode='a', encoding=None, delay=False)
    handler.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s %(levelname)s [%(name)s] %(message)s')
    handler.setFormatter(formatter)

    log.addHandler(handler)
    log.info('========================================================');

    # redirect stdout/stderr to log file - not working with tensorflow
    logger_stdout = sys.stdout
    logger_stderr = sys.stderr
    #sys.stdout = LoggerWriter(log.debug)
    #sys.stderr = LoggerWriter(log.warning)
    return log

def logger_close():
    sys.stdout = logger_stdout
    sys.stderr = logger_stderr

### mtime ###

def mtime_begin():
    return int(time.time() * 1000)

def mtime_delta(t):
    # return time difference in milliseconds
    return int(mtime_begin() - t)

def mtime_delta2(t1, t2):
    return int(t2 - t1)

def mtime_end(ss, t):
    logger.debug('visionn_server::mtime(): m="' + ss + '", dt=' + str(mtime_delta(t)))


### sock ###

def sock_listen(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    logger.debug('visionn_server::sock_listen(): msg="socket created..."')

    try:
        s.bind((host, port))
    except socket.error as msg:
        logger.error('visionn_server::sock_listen(): msg="bind failed..."')
        sys.exit()

    logger.debug('visionn_server::sock_listen(): msg="socket bind complete...", host="' + host + '", port=' + str(port))

    # Start listening on socket
    s.listen()
    logger.debug('visionn_server::sock_listen(): msg="socket now listening..."')
    return s

def sock_recv_skip(conn, chr):
    while True:
        chunk = conn.recv(1) 
        if chunk == b'':
            logger.error('visionn_server::sock_recv_skip(): msg="error1!"')
            return -1
        if chunk == chr:
            return 0

def sock_recv_head(conn, msglen):
    if sock_recv_skip(conn, b'{') < 0:
        logger.error('visionn_server::sock_recv_head(): msg="error1!"')
        return b''
    chunks = [b'{']
    bytes_recd = 1
    while bytes_recd < msglen:
        chunk = conn.recv(min(msglen - bytes_recd, VISIONN_RECV_LEN))
        if chunk == b'':
            logger.error('visionn_server::sock_recv_head(): msg="error2!"')
            return b''
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)
    return b''.join(chunks)

def sock_recv_img(conn):
    t0 = mtime_begin()
    if sock_recv_skip(conn, b'[') < 0:
        logger.error('visionn_server::sock_recv_img(): msg="error1!"')
        return b''
    chunks = []
    bytes_recd = 0
    msglen = 7
    while bytes_recd < msglen:
        chunk = conn.recv(min(msglen - bytes_recd, VISIONN_RECV_LEN))
        if chunk == b'':
            logger.error('visionn_server::sock_recv_img(): msg="error2!"')
            return b''
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)
    ss = b''.join(chunks)
    buflen = int(ss.decode('UTF-8'))
    #print('buflen: {}'.format(buflen))
    if (buflen < 0) or (buflen > 1999999):
        logger.error('visionn_server::sock_recv_img(): msg="error3!"')
        return b''

    chunk = conn.recv(2)
    if chunk == b'':
        logger.error('visionn_server::sock_recv_img(): msg="error4!"')
        return b''

    t1 = mtime_begin()
    chunks = []
    bytes_recd = 0
    while bytes_recd < buflen:
        chunk = conn.recv(min(buflen - bytes_recd, VISIONN_RECV2_LEN))
        if chunk == b'':
            logger.error('visionn_server::sock_recv_img(): msg="error5!"')
            return b''
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)

    t2 = mtime_begin()
    buf0 = b''.join(chunks)
    buf = np.fromstring(buf0, dtype='uint8')

    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)    #cv2.CV_LOAD_IMAGE_COLOR)

    t3 = mtime_begin()
    #cv2.imwrite('testing_image.png', img)
    sock_recv_skip(conn, b'}')

    logger.debug('visionn_server::sock_recv_img(): buflen=' + str(buflen) + ', dt1=' + str(mtime_delta2(t0, t1)) + ', dt2=' + str(mtime_delta2(t1, t2)) + ', dt3=' + str(mtime_delta2(t2, t3)) + ', dt4=' + str(mtime_delta(t3)))
    return img

def sock_send_hello(conn):
    resp = '{"VISIONN_HELLO":"server"}\n'
    conn.sendall(resp.encode('UTF-8'))

def sock_send_imprs(conn, pred):
    t0 = mtime_begin()
    #print("pred.shape: {}".format(pred.shape))    # pred.shape = (480, 640, 1)

    #buf = cv2.imencode('.png', pred, [cv2.IMWRITE_PNG_COMPRESSION, 0])[1].tostring()
    buf = cv2.imencode('.bmp', pred)[1].tostring()

    t1 = mtime_begin()
    buflen = len(buf);

    resp1 = '{"VISIONN_IMPRS":[' + '{:07d}'.format(buflen) + ',"'
    resp2 = '"]}\n'
    #print(resp1 + resp2)
    conn.sendall(resp1.encode('UTF-8'))
    conn.sendall(buf)
    conn.sendall(resp2.encode('UTF-8'))

    logger.debug('visionn_server::sock_send_imprs(): buflen=' + str(buflen) + ', dt1=' + str(mtime_delta2(t0, t1)) + ', dt2=' + str(mtime_delta(t1)))

### process ###

def process_cmd(conn):
    global pred

    t = mtime_begin()
    cmd0 = sock_recv_head(conn, VISIONN_CMD_LEN)
    if cmd0 == b'':
        return -1
    mtime_end('process_cmd.wait', t)
    cmd = cmd0.decode('UTF-8')
    #print('recv: {}'.format(cmd.replace("\n","").replace("\r","")))

    t2 = mtime_begin()
    if cmd == VISIONN_CMD_HELLO:
        #logger.info('visionn_server::process_cmd(): msg="HELLO request start"')
        sock_recv_skip(conn, b'}')
        logger.info('visionn_server::process_cmd(): msg="HELLO request received", dt=' + str(mtime_delta(t2)))
        sock_send_hello(conn)
        logger.info('visionn_server::process_cmd(): msg="HELLO response sent"')
    elif cmd == VISIONN_CMD_IMPRQ:
        #print('imprq')
        logger.info('visionn_server::process_cmd(): msg="IMPRQ request start"')
        img = sock_recv_img(conn)
        logger.info('visionn_server::process_cmd(): msg="IMPRQ request received", dt=' + str(mtime_delta(t2)))
        if not VISIONN_MOCK:
            t3 = mtime_begin()
            pred = visionn_model.model_predict(img)
            #cv2.imwrite('testing_mask.png', pred)
            logger.info('visionn_server::process_cmd(): msg="IMPRQ model_predict() finished", dt=' + str(mtime_delta(t3)))
        t4 = mtime_begin()
        sock_send_imprs(conn, pred)
        logger.info('visionn_server::process_cmd(): msg="IMPRQ response sent", dt=' + str(mtime_delta(t4)))
    else:
        logger.error('visionn_server::process_cmd(): msg="error: unknown command!"')
        return -1
    return 0

def process_conn(s):
    logger.info('visionn_server::process_conn(): msg="accepting conections..."')
    conn, addr = s.accept()
    logger.info('visionn_server::process_conn(): msg="connection accepted", host="' + addr[0] + '", port=' + str(addr[1]))

    try:
        while True:
            if process_cmd(conn) < 0:
                break

    except Exception as e:
        logger.exception('visionn_server::process_conn(): msg="exception: ' + str(e) + '"')
    finally:
        conn.close()
        logger.info('visionn_server::process_conn(): msg="connection closed..."')

### main ###

logger = logger_init('main', '../logout/visionn_server.log')

# import visionn_model
if not VISIONN_MOCK:
    print('import visionn_model - start')
    logger.info('visionn_server::main(): msg="import visionn_model - start"')
    import visionn_model
    print('import visionn_model - finished')
    logger.info('visionn_server::main(): msg="import visionn_model - finished"')

    logger.info('visionn_server::main(): msg="visionn_model.model_init() - start..."')
    t = mtime_begin()
    visionn_model.model_init(logger)
    logger.info('visionn_server::main(): msg="visionn_model.model_init() - finished", dt=' + str(mtime_delta(t)))

# VISIONN_MOCK
if VISIONN_MOCK:
    logger.info('visionn_server::main(): msg="MOCK VISIONN implementation, DEBUG ONLY!!"');

    pred = cv2.imread('testing_mask.png', cv2.IMREAD_GRAYSCALE)    # pred.shape = (480, 640)
    pred = np.expand_dims(pred, axis=2)                            # pred.shape = (480, 640, 1)
    #print(pred)

s = sock_listen(HOST, PORT)

try:
    while True:
        process_conn(s)

except:
    e = sys.exc_info()[0]
    logger.exception('visionn_server::main(): msg="exception: ' + str(e) + '"')
finally:
    logger.info('visionn_server::main(): msg="application exit"')
    logger_close()

