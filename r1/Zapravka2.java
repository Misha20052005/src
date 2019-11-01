package r1;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;
import java.util.Vector;

import javax.xml.crypto.dsig.spec.XSLTTransformParameterSpec;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
//import lejos.remote.rcx.Serial;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.localization.MCLParticleSet;
import lejos.utility.Delay;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.I2CSensor;

public class Zapravka2 {

	static int I2CSlaveAddress = 8;
	// static int back1 = 150;
	static byte clear = 116;
	static byte alert = 117;
	static byte wellcome = (byte) 246;
	static byte good = 120;
	static byte stopFilling = 121;
	static byte startFilling = 122;
	static byte back = (byte) 251;
	static byte stop = (byte) 252;
	static byte forward = (byte) 253;
	static byte active = (byte) 254;
	static byte init = (byte) 255;
	static byte volume = 100;
	static double[] circle = new double[3];

	static byte cDy = 8;
	static byte cSy = 4;
	static byte cDz = 32;
	static byte cSz = 16;
	static byte cDx = 2;
	static byte cSx = 1;

	static byte cmd;
	static byte cmd2;

	static final int x1z = 35;
	static final int y1z = 75;
	static final int x2z = 115;
	static final int y2z = 35;
	static int raz1 = 0;
	static int raz2 = 0;
	static int raz3 = 0;
	static int d = 1000000000;
   static int zaliv = 0; 
	static int razy1 = 0;
	static int razy2 = 0;
	static int razy3 = 0;
	static int x02 = 10;
	static int y02 = 10;
	static int razx = 0;
	static int green = 0;
	static int normalno = 0;

	public static void main(String[] args) throws Exception {
		int[][] massiv = { { 45, 40, 135, 57 }, { 45, 60, 135, 42 }, { 45, 80, 135, 25 }, { 62, 40, 127, 58 },
				{ 62, 60, 124, 42 }, { 62, 80, 122, 25 }, { 80, 40, 106, 50 }, { 80, 60, 103, 43 }, { 80, 80, 100, 25 },
				{ 98, 40, 82, 62 }, { 98, 60, 81, 44 }, { 98, 80, 80, 36 }, { 115, 40, 60, 64 }, { 115, 60, 59, 45 },
				{ 115, 80, 57, 37 } };
		int g = 0;

		int num = 0;
		int numGo = 0;
		int numBack = 0;
		int numStop = 0;
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		Mat frame1 = new Mat();
		Mat frameGray = new Mat();
		Mat circles = new Mat();
		Mat result = new Mat();

		I2CSensor arduino = new I2CSensor(SensorPort.S2, I2CSlaveAddress);
		I2CSensor arduino1 = new I2CSensor(SensorPort.S1, 4);
		VideoCapture videoCapture = new VideoCapture(0);
		// Задаём ширину и высоту кадра, с которым будем работать.
		final int WIDTH = 160, HEIGHT = 120;
		videoCapture.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, WIDTH);
		videoCapture.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
		// Задаём ширину и высоту центральной области.
		// Когда за рамки этой области будет выходить объект, будем двигать робота.
		final int CENTER_WIDTH = 50, CENTER_HEIGHT = 50;
		// Считаем координаты центра кадра.
		Point imageCenter = new Point(WIDTH / 2, HEIGHT / 2);
		// Захватываем камеру для работы с видео.
		if (videoCapture.open(0))
			// Выдаём сообщение, что камера готова.
			LCD.drawString("Camera is ready.", 0, 0);
		else {
			// Выдаём сообщение, что камера не готова. Может быть не подключена?
			LCD.drawString("Camera is not ready.", 0, 0);
			Delay.msDelay(5000);
			return;
		}
		int x0 = 80;
		int y0 = 60;

		// Выводим на экран приглашение выбрать режим (обычный или отладка).
		LCD.drawString("Choose the mode:", 0, 1);
		LCD.drawString("Up - normal;", 0, 2);
		LCD.drawString("Down - debug.", 0, 3);

		boolean debug = false;
		int debugMode = 0;

		// Пока ждём выбор режима, считываем изображения с камеры.
		// arduino.sendData(0, (byte) 0);
		while (true) {
			videoCapture.read(frame1);
			if (Button.ESCAPE.isDown())
				return; // Esc - выход из программы.
			else if (Button.UP.isDown())
				break; // Кнопка "вверх" - выбран обычный режим.
			else if (Button.DOWN.isDown()) {
				debug = true; // Кнопка "вниз" - выбран режим отладки.
				break;
			}
		}

		// //Берём пробу цвета в центре кадра.

		final Scalar RED = new Scalar(0, 0, 255);
		final Scalar GREEN = new Scalar(0, 255, 0);

		ServerSocket serverSocket = null;
		Socket socket = null;
		String boundary = "Thats it folks!";
		try {
			if (debug) {
				// Выводим на экран приглашение подключиться к EV3 с помощью браузера.
				LCD.clear();
				LCD.drawString("Connect to", 0, 0);
				LCD.drawString("http://10.0.1.1:8080", 0, 1);
				// Создаём серверный сокет.
				serverSocket = new ServerSocket(8080);
				// Ждём подключения к серверу.
				socket = serverSocket.accept();
				// Клиент (браузер) подключен, выводим информацию об этом на экран.
				LCD.drawString("Connected!", 0, 2);
				// Отдаём клиенту (браузеру) HTTP-заголовок.
				writeHeader(socket.getOutputStream(), boundary);
			}

			// Вычисляем интервал цветов, близких к цвету пробы.

			// Scalar hsvMin = new Scalar(0, 0, 200);
			// Scalar hsvMax = new Scalar(359, 50, 255);
			// Scalar hsvMin = new Scalar(0, 0, 0);
			// Scalar hsvMax = new Scalar(359, 255, 50);

			GraphicsLCD lcd = BrickFinder.getDefault().getGraphicsLCD();

			MatOfByte buffer = new MatOfByte();
			int numStop1 = 0;
			while (!Button.ESCAPE.isDown()) {
				// Меняем режим отладки, если нажата клавиша Enter на модуле EV3.
				if (Button.ENTER.isDown()) {
					debugMode++;
					if (debugMode >= 4)
						debugMode = 0;
				}

				// Считываем кадр с камеры.
				if (videoCapture.read(frame1) && !frame1.empty()) {

					Point pt1 = new Point(0, y0);
					Point pt2 = new Point(160, y0);

					Point pt3 = new Point(x0, 0);
					Point pt4 = new Point(x0, 120);

					Point z1 = new Point(x1z, y1z);
					Point z2 = new Point(x2z, y1z);
					Point z3 = new Point(x1z, y2z);
					Point z4 = new Point(x2z, y2z);

					Core.line(frame1, z1, z2, RED);
					Core.line(frame1, z2, z4, RED);
					Core.line(frame1, z4, z3, RED);
					Core.line(frame1, z3, z1, RED);

					Core.line(frame1, pt1, pt2, RED);
					Core.line(frame1, pt3, pt4, RED);

					// Imgproc.blur(image, image, ksize); //Размытие отключено, для ускорения.
					// Полученный кадр формата BGR. Переводим его в формат HSV.
					if (circle[0] == 0 & num == 0) {
						System.out.println("Welcome!!");
						doWellcome(arduino1);
						num = 1;

					}
					// Ищем цвета в нужном нам интервале. В результате получится чёрно-белое
					// изображение,
					// где белый цвет обозначает цвет, попавший в интервал, а чёрный - всё1
					// остальное.
					// circle[0] = 0;
					// circle[1] = 0;
					// circle[2] = 0;
					double dmishax = 5;
					double dmishay = 5;
					int l = 0;
					Point center = new Point(circle[0], circle[1]);
					// circles.setTo(new MatOfInt(0, 0, 0));
					// circle = 0;
                    if(normalno == 0) {
                    	boolean b;
					Imgproc.cvtColor(frame1, frameGray, Imgproc.COLOR_BGR2GRAY);
					Imgproc.HoughCircles(frameGray, circles, Imgproc.CV_HOUGH_GRADIENT, 2, frameGray.rows() / 2);
					
					
					for (int i = 0, r = circles.rows(); i < r; i++) {
						for (int j = 0, c = circles.cols(); j < c; j++) {
							circle = circles.get(i, j);

							Core.circle(frame1, center, (int) circle[2], RED);
							l = 1;

						}
					}
                    }
					

					// Ищем контуры, т.е. объекты нужного нам цвета.

					// arduino.sendData(wellcome, wellcome);
					// System.out.println("Welcome!!");
					// break;
					double x3 = center.x - dmishax;
					double x4 = center.x + dmishax;
					double y3 = center.y - dmishay;
					double y4 = center.y + dmishay;
					Point pt5 = new Point(x3, center.y);
					Point pt6 = new Point(x4, center.y);
					Point pt7 = new Point(center.x, y3);
					Point pt8 = new Point(center.x, y4);
					// Если включена отладка и режим отладки 0 (contour)...
					if (l == 1) {

						// double[] sampleColor = getSampleColor(videoCapture, WIDTH, HEIGHT, center.x,
						// center.y, 10, 10);

						Core.line(frame1, pt5, pt6, RED);
						Core.line(frame1, pt7, pt8, RED);

						if ((center.x > x2z) & (numGo == 0)) {
//							System.out.println("G0");
							doBack(arduino1);
							numGo = 1;
							numBack = 0;
							numStop = 0;
							numStop1 = 0;
						}

						if ((center.x < x1z) & (center.x != 0) & (numBack == 0)) {
//							System.out.println("Back");
							doForward(arduino1);
							numGo = 0;
							numStop = 0;
							numBack = 1;
							numStop1 = 0;
						}

						if ((center.x > x1z) & (center.x < x2z) & (numStop1 == 0)) {

//							System.out.println("Stop");
							doStop(arduino1);
							numGo = 0;
							numStop1 = 1;
							numBack = 0;

						}

						
					}
					if ((center.x > x1z) & (center.x < x2z) & green == 0) {

//						System.out.println("3");
						Core.line(frame1, pt5, pt6, GREEN);
						Core.line(frame1, pt7, pt8, GREEN);
						Core.circle(frame1, center, (int) circle[2], GREEN);
					}
					
					if ((center.x > x1z) & (center.x < x2z) & (numStop == 0)) {

//						System.out.println("Stop");
						// doStop(arduino1);
						numGo = 0;

						numBack = 0;
						// razx = (int) (160 - center.x);
						// System.out.println(razx);
						int d = 0;
						int min = 1000000;
						for (int i = 0; i < 14; i++) {

							d = (int) Math.sqrt(
									Math.pow((center.x - massiv[i][0]), 2) + Math.pow((center.y - massiv[i][1]), 2));
							if (min > d) {
								min = d;
								g = i;
								// min = d^2;
							}
						}
						
						if (Button.LEFT.isDown()) {
							arduino.sendData(-127, (byte) (massiv[g][2] - 120));
							arduino.sendData(-20, (byte) (massiv[g][3] - 120));
							normalno = 1;
							
//							doStartFilling(arduino1);
//							doFilling(arduino1, (byte) 100);
//							doStopFilling(arduino1);
//							
//							green = 1;
//							normalno = 1;
						}
						
						
						if(Button.ENTER.isDown()) {
							arduino.sendData(-127, (byte) -120);
							arduino.sendData(-120, (byte) -120);
							numStop = 1;
						}

						// System.out.println(min);

					}

				}

				// Если включена отладка и режим отладки 0 (contour),
				// отдаём браузеру кадр, полученный с камеры,
				// с подрисованными контуром и кругом.
				if (debug && debugMode == 0) {
					Highgui.imencode(".jpeg", frame1, buffer);
					writeJpg(socket.getOutputStream(), buffer, boundary);
				}

			}
			lcd.clear();
		} finally {
			// Закрываем сокеты.
			if (socket != null)
				socket.close();
			if (serverSocket != null)
				serverSocket.close();
		}
	}

	// Функция получает пробу цвета с камеры.

	// Функция отдаёт браузеру HTTP-заголовок.
	private static void writeHeader(OutputStream stream, String boundary) throws IOException {
		stream.write(("HTTP/1.0 200 OK\r\n" + "Connection: close\r\n" + "Max-Age: 0\r\n" + "Expires: 0\r\n"
				+ "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
				+ "Pragma: no-cache\r\n" + "Content-Type: multipart/x-mixed-replace; " + "boundary=" + boundary + "\r\n"
				+ "\r\n" + "--" + boundary + "\r\n").getBytes());
	}

	
	private static void writeJpg(OutputStream stream, MatOfByte image, String boundary) throws IOException {
		byte[] imageBytes = image.toArray();
		stream.write(
				("Content-type: image/jpeg\r\n" + "Content-Length: " + imageBytes.length + "\r\n" + "\r\n").getBytes());
		stream.write(imageBytes);
		stream.write(("\r\n--" + boundary + "\r\n").getBytes());
	}

	static void doClear(I2CSensor arduino) {
		arduino.sendData(116, clear);
		System.out.println(init);
	}

	static void doAlert(I2CSensor arduino) {
		arduino.sendData(117, alert);
		System.out.println(alert);
	}

	static void doWellcome(I2CSensor arduino) {
		arduino.sendData(118, wellcome);
		System.out.println(wellcome);
	}

	static void doGood(I2CSensor arduino) {
		arduino.sendData(120, good);
		System.out.println(good);
	}

	static void doStopFilling(I2CSensor arduino) {
		arduino.sendData(121, stopFilling);
		System.out.println(stopFilling);
	}

	static void doStartFilling(I2CSensor arduino) {
		arduino.sendData(122, startFilling);
		System.out.println(startFilling);
	}

	static void doBack(I2CSensor arduino) {
		arduino.sendData(123, back);
		System.out.println(back);
	}

	static void doStop(I2CSensor arduino) {
		arduino.sendData(124, stop);
		System.out.println(stop);
	}

	static void doForward(I2CSensor arduino) {
		arduino.sendData(125, forward);
		System.out.println(forward);
	}

	static void doActive(I2CSensor arduino) {
		arduino.sendData(126, active);
		System.out.println(active);
	}

	static void doInit(I2CSensor arduino) {
		arduino.sendData(127, init);
		System.out.println(init);
	}

	static void doFuel(I2CSensor arduino, byte volume) {
		arduino.sendData(volume, volume);
		System.out.println(volume);
	}

	static void doFilling(I2CSensor arduino, byte volume) {
		arduino.sendData(volume, volume);
	}

	static void otpravka(I2CSensor arduino, byte cDy, byte cSy, byte cDz, byte cSz, byte cDx, byte cSx) {
		cmd = (byte) (cDy + cSy + cDz + cSz + cDx + cSx);
		arduino.sendData(cmd, cmd);
	}

}
