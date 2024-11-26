package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;

public class PlayerID implements IPlayer, IAuto{
    String name;

    public PlayerID(String name) {
        this.name = name;
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        return null;
    }

    @Override
    public void timeout() {
        
    }

    @Override
    public String getName() {
        return "PlayerID(" + name + ")";
    }
}
