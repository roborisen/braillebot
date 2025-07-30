braillebot.setupBrailleBot()
basic.forever(function () {
    braillebot.readColorSensor()
    braillebot.showColorKey(braillebot.getColorKey())
    if (braillebot.getColorKey() == 1) {
        braillebot.playTwoNotes(braillebot.Note.Sol, braillebot.Note.Si, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnRight)
        braillebot.turnRight()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.getColorKey() == 3) {
        braillebot.playTwoNotes(braillebot.Note.Sol, braillebot.Note.Mi, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnLeft)
        braillebot.turnLeft()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.getColorKey() == 6) {
        braillebot.playTwoNotes(braillebot.Note.Sol, braillebot.Note.Sol, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.UTurn)
        braillebot.uTurn()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.getColorKey() == 7) {
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.getColorKey() == 4) {
        braillebot.setEchoOn()
        braillebot.playTwoNotes(braillebot.Note.Do, braillebot.Note.Do, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.getColorKey() == 8) {
        braillebot.playTwoNotes(braillebot.Note.Do, braillebot.Note.HighDo, braillebot.Action.Action)
        braillebot.gripperOpenBlock(braillebot.Opening.MovingOpen)
    } else if (braillebot.getColorKey() == 5) {
        braillebot.playTwoNotes(braillebot.Note.HighDo, braillebot.Note.Do, braillebot.Action.Action)
        braillebot.gripperCloseBlock(braillebot.Closing.MovingClose)
    } else if (braillebot.getColorKey() == 2) {
        braillebot.playTwoNotes(braillebot.Note.HighDo, braillebot.Note.HighDo, braillebot.Action.Stop)
    } else {

    }
})
