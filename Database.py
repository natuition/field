import psycopg2
import datetime
from datetime import datetime
from psycopg2.extensions import AsIs
from socketForRTK.Server import Server


class Database:
	"""
		Class managing the insertion of data in the postgresql database.
	"""

	def __init__(self, host, dbname, user, password, port, robotSerialNumber, serveurPort):
		"""
			Inits Database with the host of the database, its name, the database-user name, its password and its port of connexion.\n
			The roboSerialNumber is optionnal and serveurPort is the connection port of RTK server.\n
			Create an instance of Server and APIWeather.

			:param host: Path of the database host
			:param dbname: Name of the database
			:param user: Name of the user used for the connection to database
			:param password: Password of the database user
			:param port: Connection port for the database
			:param robotSerialNumber: optional, used for init the global variable of the robot serial number
			:param serveurPort: Connection port of the server that send gps coordinates

			**Authors of this class :** SOULLARD Thomas and TOURNEUR Hugo and CHENNUAUD Emmanuel and LAMBERT Vincent. \n
		"""
		self.host = host
		self.dbName = dbname
		self.user = user
		self.password = password
		self.port = port
		self.robotSerialNumber = robotSerialNumber
		self.sessionID = None
		self.server = Server(int(serveurPort))
		self.lastCoordinate = None

	def insertRobot(self, serialNumber):

		"""
			Insert a robot's informations in the database. \n
			This method :
				* obtain the serial number of the robot,
				* send it in the Database by a POSTGRESQL Request.

			:param serialNumber: the serial number of a robot.
		"""

		# INSERT INTO robot(serial_number) SELECT N189563 WHERE NOT EXISTS (SELECT serial_number FROM robot WHERE serial_number = N189563);

		sql = 'INSERT INTO robot(serial_number) SELECT \'{}\' WHERE NOT EXISTS (SELECT serial_number FROM robot WHERE serial_number = \'{}\');'.format(
			self.robotSerialNumber, self.robotSerialNumber)
		conn = None
		try:
			# connect to the PostgreSQL database
			conn = psycopg2.connect(dbname=self.dbName, user=self.user, host=self.host, password=self.password)
			# create a new cursor
			cur = conn.cursor()
			# execute the INSERT statement
			cur.execute(sql)
			# commit the changes to the database
			conn.commit()
			# close communication with the database
			cur.close()
		except(Exception, psycopg2.DatabaseError) as error:
			print("[Database]")
			print(error)
		finally:
			if conn is not None:
				conn.close()

	def startSession(self):

		"""
			Send to the database the start informations of the robot's session. \n
			This method : 
				* format the start date, hour and coordinates,
				* put them in a POSTGRESQL request,
				* send them in the database.
		"""

		now = datetime.now().time()
		DateSession = str(datetime.now())
		Begin_Hour = str(now.hour) + ":" + str(now.minute) + ":" + str(now.second)
		coordinate = self.server.getLocation()
		coordinateLong = coordinate['latitude']
		coordinateLat = coordinate['longitude']
		Start_Position = AsIs("'(%s,%s)'" % (coordinateLat, coordinateLong))
		sql = """INSERT INTO "session"(date,Start_Position,Begin_Hour,robot)
				VALUES(%s,%s,%s,%s) RETURNING id"""

		conn = None
		try:
			# connect to the PostgreSQL database
			conn = psycopg2.connect(dbname=self.dbName, user=self.user, host=self.host, password=self.password)
			# create a new cursor
			cur = conn.cursor()
			# execute the INSERT statement
			cur.execute(sql, (DateSession, Start_Position, Begin_Hour, self.robotSerialNumber))
			# get the generated id back
			self.sessionID = cur.fetchone()[0]
			# commit the changes to the database
			conn.commit()
			# close communication with the database
			cur.close()
		except (Exception, psycopg2.DatabaseError) as error:
			print("[Database]")
			print(error)
		finally:
			if conn is not None:
				conn.close()

	def endSession(self):
		"""
			Complete information of a session with the end hour. \n
			This method :
				* recover the end hour of a robot session,
				* format the hour,
				* put it in a POSTGRESQL request,
				* send it in the database.
		"""

		now = datetime.now().time()
		End_Hour = str(now.hour) + ":" + str(now.minute) + ":" + str(now.second)

		sql = """UPDATE session
				SET End_Hour = %s
				WHERE id = %s"""
		conn = None
		try:
			# connect to the PostgreSQL database
			conn = psycopg2.connect(dbname=self.dbName, user=self.user, host=self.host, password=self.password)
			# create a new cursor
			cur = conn.cursor()
			# execute the INSERT statement
			cur.execute(sql, (End_Hour, self.sessionID))
			# commit the changes to the database
			conn.commit()
			# close communication with the database
			cur.close()
		except (Exception, psycopg2.DatabaseError) as error:
			print("[Database]")
			print(error)
		finally:
			if conn is not None:
				conn.close()

	def insertResults(self, angle):

		"""
			Insert the results of an angle measure on the robot in the database with the actual weather. \n
			This method :
				* recover and format the hour of measure,
				* recover the coordinates where the robot was,
				* call the APIWeather class to get the weather,
				* recover the angle measured,
				* send them in the Database with a POSTGRESQL request.
				* calculate the robot orientation vector and send it to database (qgis)

			:param angle: the angle measured by the angular captor
		"""

		weather = "Wether not specified"
		temperature = 0.0
		humidity = 0
		coordinate = self.server.getLocation()
		coordinateLat = coordinate['latitude']
		coordinateLong = coordinate['longitude']
		idResultat = None
		vector_robot_direction = None
		sql2 = None

		if self.lastCoordinate is not None:
			latCompass = coordinateLat - self.lastCoordinate['latitude']
			longCompass = coordinateLong - self.lastCoordinate['longitude']
			vector_robot_direction = AsIs("'(%s,%s)'" % (longCompass, latCompass))

		# A supprimer quand il y aura rtk et weather
		now = datetime.now().time()
		coordinateStr = AsIs("'(%s,%s)'" % (coordinateLong, coordinateLat))
		time_hour = str(now.hour) + ":" + str(now.minute) + ":" + str(now.second)

		sql = """INSERT INTO resultat(angle,coordinates,timer_hour,weather,humidity,temperature,session)
				VALUES(%s,%s,%s,%s,%s,%s,%s) RETURNING id;"""

		if vector_robot_direction is not None:
			sql2 = """INSERT INTO qgis(id_resultat,coordinates,angle,session,vector_robot_direction)
				VALUES(%s,%s,%s,%s,%s);"""
		else:
			sql2 = """INSERT INTO qgis(id_resultat,coordinates,angle,session)
				VALUES(%s,%s,%s,%s);"""
		try:
			# connect to the PostgreSQL database
			conn = psycopg2.connect(dbname=self.dbName, user=self.user, host=self.host, password=self.password)
			# create a new cursor
			cur = conn.cursor()
			# execute the INSERT statement
			cur.execute(sql, (angle, coordinateStr, time_hour, weather, humidity, temperature, self.sessionID,))

			idResultat = cur.fetchone()[0]

			conn.commit()

			if vector_robot_direction is not None:
				cur.execute(sql2, (idResultat, coordinateStr, angle, self.sessionID, vector_robot_direction,))
			else:
				cur.execute(sql2, (idResultat, coordinateStr, angle, self.sessionID,))

			# commit the changes to the database
			conn.commit()
			# close communication with the database
			cur.close()
		except (Exception, psycopg2.DatabaseError) as error:
			print("[Database]")
			print(error)
		finally:
			if conn is not None:
				conn.close()

		self.lastCoordinate = dict(coordinate)

	def startServer(self):
		"""
			This method start the server for reiceve the location.
		"""
		self.server.start()

	def stopServer(self):
		"""
			This method stop the server.
		"""
		self.server.exit()
