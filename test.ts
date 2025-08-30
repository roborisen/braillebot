braillebot.setupBrailleBot()
basic.forever(function () {
    braillebot.readColorSensor()
    braillebot.showColorKey(braillebot.getColorKey())
    if (braillebot.isColorDetected(braillebot.inKey.RED_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnRight)
        braillebot.turnRight()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.BLUE_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnLeft)
        braillebot.turnLeft()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.YELLOW_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.UTurn)
        braillebot.uTurn()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.ORANGE_KEY)) {
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.CYAN_KEY)) {
        braillebot.setEchoOn()
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.PINK_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.gripperOpenBlock(braillebot.Opening.MovingOpen)
    } else if (braillebot.isColorDetected(braillebot.inKey.VIOLET_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.gripperCloseBlock(braillebot.Closing.MovingClose)
    } else if (braillebot.isColorDetected(braillebot.inKey.GREEN_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Stop)
    } else {

    }
})
