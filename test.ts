braillebot.setupBrailleBot()
music.play(music.builtinPlayableSoundEffect(soundExpression.hello), music.PlaybackMode.UntilDone)
basic.forever(function () {
    braillebot.readColorSensor()
    braillebot.showColorKey(braillebot.readColorKey())
    if (braillebot.isColorDetected(braillebot.InKey.RedKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.B4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnRight)
        braillebot.turnRight()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.BlueKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.E4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnLeft)
        braillebot.turnLeft()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.YellowKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.G4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.UTurn)
        braillebot.uTurn()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.OrangeKey)) {
        basic.pause(500)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.CyanKey)) {
        braillebot.setEchoOn()
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.PinkKey)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C5, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.GripperOpen)
        braillebot.gripperOpenBlock(braillebot.Opening.MovingOpen)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.VioletKey)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.GripperClose)
        braillebot.gripperCloseBlock(braillebot.Closing.MovingClose)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.GreenKey)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C5, braillebot.Action.Stop)
        braillebot.showIcon(braillebot.Icons.Stop)
    } else {

    }
})
