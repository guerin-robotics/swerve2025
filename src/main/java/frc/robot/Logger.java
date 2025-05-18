package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.lang.StackWalker;

public class Logger {
    public enum Level {
        DEBUG(4), 
        INFO(3), 
        WARN(2), 
        ERROR(1), 
        OFF(0);
        
        public final int value;

        Level(int value) {
            this.value = value;
        }
    }

    private static Level currentLevel = Level.INFO;

    public static void setLevel(Level level) {
        System.out.println("[ALWAYS] Logger::setLevel " + level);
        currentLevel = level;
    }

    private static boolean shouldLog(Level level) {
        return currentLevel.value >= level.value;
    }

    // StackWalker and cache for caller info to avoid reflection overhead
    private static final StackWalker STACK_WALKER =
        StackWalker.getInstance(StackWalker.Option.RETAIN_CLASS_REFERENCE);
    private static final ConcurrentMap<String, String> callerInfoCache =
        new ConcurrentHashMap<>();

    /**
     * Returns a short "[Class#method] " prefix for the calling method,
     * caching results to avoid repeated reflection.
     */
    private static String getCallerInfo() {
        StackWalker.StackFrame frame = STACK_WALKER.walk(frames ->
            frames.filter(f -> !f.getClassName().equals(Logger.class.getName()))
                  .findFirst().orElse(null));
        if (frame == null) {
            return "";
        }
        String key = frame.getClassName() + "#" + frame.getMethodName();
        return callerInfoCache.computeIfAbsent(key, k ->
            "[" + frame.getDeclaringClass().getSimpleName()
            + "#" + frame.getMethodName() + "] ");
    }

    public static void debug(String msg) {
        if (shouldLog(Level.DEBUG)) {
            System.out.println("[DEBUG] " + getCallerInfo() + msg);
        }
    }

    public static void info(String msg) {
        if (shouldLog(Level.INFO)) {
            System.out.println("[INFO] " + getCallerInfo() + msg);
        }
    }

    public static void warn(String msg) {
        if (shouldLog(Level.WARN)) {
            DriverStation.reportWarning(msg, false);
            System.out.println("[WARN] " + getCallerInfo() + msg);
        }
    }

    public static void error(String msg) {
        if (shouldLog(Level.ERROR)) {
            DriverStation.reportError(msg, false);
            System.err.println("[ERROR] " + getCallerInfo() + msg);
        }
    }

    /**
     * Replace "{}" placeholders in the template with the provided arguments.
     */
    private static String formatTemplate(String template, Object... args) {
        if (template == null || args == null || args.length == 0) {
            return template;
        }
        StringBuilder sb = new StringBuilder();
        int start = 0;
        for (Object arg : args) {
            int idx = template.indexOf("{}", start);
            if (idx == -1) {
                break;
            }
            sb.append(template, start, idx);
            sb.append(arg);
            start = idx + 2;
        }
        sb.append(template, start, template.length());
        return sb.toString();
    }

    public static void debug(String template, Object... args) {
        if (shouldLog(Level.DEBUG)) {
            System.out.println("[DEBUG] " + getCallerInfo() + formatTemplate(template, args));
        }
    }

    public static void info(String template, Object... args) {
        if (shouldLog(Level.INFO)) {
            System.out.println("[INFO] " + getCallerInfo() + formatTemplate(template, args));
        }
    }

    public static void warn(String template, Object... args) {
        if (shouldLog(Level.WARN)) {
            String msg = formatTemplate(template, args);
            DriverStation.reportWarning(msg, false);
            System.out.println("[WARN] " + getCallerInfo() + msg);
        }
    }

    public static void error(String template, Object... args) {
        if (shouldLog(Level.ERROR)) {
            String msg = formatTemplate(template, args);
            DriverStation.reportError(msg, false);
            System.err.println("[ERROR] " + getCallerInfo() + msg);
        }
    }
}