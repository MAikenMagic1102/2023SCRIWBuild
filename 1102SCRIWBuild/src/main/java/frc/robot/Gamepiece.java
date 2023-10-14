// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Gamepiece {
    public static enum GamepieceType {
        Nothing, Cone, Cube;
    }
    public static GamepieceType currentGamepiece;

    public static void setGamepiece(GamepieceType requestedGamepiece){
        currentGamepiece = requestedGamepiece;
    }

    public static GamepieceType getGamepiece(){
        return currentGamepiece;
    }

    public static void toggleGamePiece(){
    }
}
