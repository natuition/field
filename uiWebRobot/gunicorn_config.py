def worker_exit(server, worker):
    server.log.info("Gunicorn worker_exit: calling uiWebRobot.exit()")

    try:
        from uiWebRobot.application import uiWebRobot
        uiWebRobot.exit()
    except Exception as e:
        server.log.error("Gunicorn worker_exit error: %s", e)