package frc.robot;

// import java.lang.Integer;

public class VisionTarget {
    public String id;
    public int x;
    public int y;
    public int height;
    public int width;

    public VisionTarget(String str) {
        String[] tokens = str.split(",");
        id = tokens[0];
        try {
            x = Integer.parseInt(tokens[1]);
            y = Integer.parseInt(tokens[2]);
            height = Integer.parseInt(tokens[3]);
            width = Integer.parseInt(tokens[4]);
        } catch(NumberFormatException ex) {
        }
    }

    public VisionTarget(String ida,
                        int xa,
                        int ya,
                        int heighta,
                        int widtha) {
        id = ida;
        x = xa;
        y = ya;
        height = heighta;
        width = widtha;
    }

    public void printSystemOut() {
        System.out.println("Id = " + id);
        System.out.println("x = " + x);
        System.out.println("y = " + y);
        System.out.println("height = " + height);
    }
}