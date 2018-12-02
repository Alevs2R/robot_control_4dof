package sample;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.stage.Stage;
import py4j.GatewayServer;


import java.util.Timer;
import java.util.TimerTask;

public class Main extends Application {

    boolean rotateLeft, rotateRight, heightLeft, heightRight, stretchLeft, stretchRight, handrotLeft, handrotRight, gripPinch, gripSpread;

    Driver driver = new Driver();

    int step = 3;

    class DeviceTask extends TimerTask {
        public void run() {
            if (rotateLeft) driver.setRotation(driver.rotation + step);
            if (rotateRight) driver.setRotation(driver.rotation - step);
            if (heightLeft) driver.setHeight(driver.height - step);
            if (heightRight) driver.setHeight(driver.height + step);
            if (stretchLeft) driver.setStretch(driver.stretch - step);
            if (stretchRight) driver.setStretch(driver.stretch + step);
            if (handrotLeft) driver.setHandrot(driver.handrot - 7);
            if (handrotRight) driver.setHandrot(driver.handrot + 7);
            if (gripPinch) {
                driver.setGrip(true, true);
            } else if (gripSpread) {
                driver.setGrip(true, false);
            } else {
                driver.setGrip(false, false);
            }
            driver.sendData();
        }
    }

    void buttonChange(KeyCode code, boolean newValue) {
        System.out.println(String.format("%s %b", code.getName(), newValue));
        switch (code) {
            case UP:    stretchLeft = newValue; break;
            case DOWN:  stretchRight = newValue; break;
            case LEFT:  rotateLeft  = newValue; break;
            case RIGHT: rotateRight  = newValue; break;
            case A: handrotLeft = newValue; break;
            case D: handrotRight = newValue; break;
            case W: heightLeft = newValue; break;
            case S: heightRight = newValue; break;
            case SHIFT: gripSpread = newValue; break;
            case SPACE: gripPinch = newValue; break;
        }
    }

    @Override
    public void start(Stage primaryStage) throws Exception{
        Parent root = FXMLLoader.load(getClass().getResource("sample.fxml"));
        primaryStage.setTitle("Hello World");
        Scene scene = new Scene(root, 300, 275);

        scene.setOnKeyPressed(event -> {
            buttonChange(event.getCode(), true);
        });

        scene.setOnKeyReleased(event -> {
            buttonChange(event.getCode(), false);

        });

        primaryStage.setScene(scene);
        primaryStage.show();

        Timer timer = new Timer();
      timer.scheduleAtFixedRate(new DeviceTask (), 0,40);

        // app is now the gateway.entry_point
        GatewayServer server = new GatewayServer(driver);
        server.start();
    }


    public static void main(String[] args) {
        launch(args);
    }
}
