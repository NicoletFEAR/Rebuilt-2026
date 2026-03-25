package frc.robot.containers;

public enum BotEnum {

    TUSK("tusk", 4786),
    HADES("hades", 4785),
    KITBOT("kitbot", 4784);

    private final String name;
    private int teamNumber;
    
    BotEnum(String name, int teamNumber) {
        this.name = name;
        this.teamNumber = teamNumber;
    }

    public String getName() {
        return name;
    }

    public int getTeamNumber() {
        return teamNumber;
    }

    public static BotEnum fromString(String name) {
        for (BotEnum bot : BotEnum.values()) {
            if (bot.name.equals(name)) {
                return bot;
            }
        }
        return null;
    }

    public static BotEnum fromTeamNumber(int teamNumber) {
        for (BotEnum bot : BotEnum.values()) {
            if (bot.teamNumber == teamNumber) {
                return bot;
            }
        }
        return null;
    }
}