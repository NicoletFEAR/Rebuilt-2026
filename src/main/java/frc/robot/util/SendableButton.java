package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SendableButton {
    private BooleanPublisher m_publisher;
    private BooleanSubscriber m_subscriber;
    private BooleanTopic m_topic;

    public SendableButton(String topicName, Runnable onPressed) {
        m_topic = NetworkTableInstance.getDefault().getBooleanTopic(topicName);
        m_publisher = m_topic.publish();
        m_publisher.setDefault(false);
        m_subscriber = m_topic.subscribe(false);

        if (onPressed != null) {
            new Trigger(() -> m_subscriber.get(false)).onTrue(Commands.runOnce(() -> {
                m_publisher.set(false);
                onPressed.run();
            }).ignoringDisable(true));
        }

    }

    public SendableButton(String topicName, Command onPressed, boolean ignoringDisable) {
        this(topicName, null);

        new Trigger(() -> m_subscriber.get(false))
            .onTrue(Commands.runOnce(() -> m_publisher.set(false)).alongWith(onPressed)
                .ignoringDisable(ignoringDisable));
    }
}
