braillebot.setupBrailleBot()
music.play(music.builtinPlayableSoundEffect(soundExpression.hello), music.PlaybackMode.UntilDone)
basic.forever(function () {
    braillebot.readColorSensor()
    braillebot.showColorKey(braillebot.getColorKey())
    if (braillebot.isColorDetected(braillebot.inKey.RED_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.B4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnRight)
        braillebot.turnRight()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.BLUE_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.E4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnLeft)
        braillebot.turnLeft()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.YELLOW_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.G4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.UTurn)
        braillebot.uTurn()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.ORANGE_KEY)) {
        basic.pause(500)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.CYAN_KEY)) {
        braillebot.setEchoOn()
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.PINK_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C5, braillebot.Action.Action)
        braillebot.gripperOpenBlock(braillebot.Opening.MovingOpen)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.VIOLET_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.gripperCloseBlock(braillebot.Closing.MovingClose)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.inKey.GREEN_KEY)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C5, braillebot.Action.Stop)
    } else {

    }
})
