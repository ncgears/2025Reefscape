package frc.robot.classes;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.NCDebug;

public class NCOrchestra {
    private static NCOrchestra instance;
    private String m_music = "";
    private Orchestra m_orchestra = new Orchestra();
    private boolean flagChanged = false;
    private boolean hasAddedInstruments = false;
    public boolean isPlaying = false;

    /**
	 * Returns the instance of the NCOrchestra class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return NCOrchestra instance
	 */
    public static NCOrchestra getInstance() {
		if (instance == null) instance = new NCOrchestra();
		return instance;
	}


    public StatusCode apply(TalonFX... motorsToApply) {
        StatusCode ret = StatusCode.OK;

        if (!hasAddedInstruments) {
            ret = addInstruments(motorsToApply);
            hasAddedInstruments = true;
        }

        if (flagChanged) {
            NCDebug.Debug.debug("Orchestra: load song music/"+m_music);
            ret = m_orchestra.loadMusic("music/"+m_music);
            flagChanged = false;
        }

        return ret;
    }

    public boolean isPlaying() { return isPlaying; }

    public StatusCode update() {
        StatusCode ret = StatusCode.OK;
        if (flagChanged) {
            NCDebug.Debug.debug("Orchestra: load song music/"+m_music);
            ret = m_orchestra.loadMusic("music/"+m_music);
            flagChanged = false;
        }
        return ret;
    }

    public StatusCode play() {
        NCDebug.Debug.debug("Orchestra: play");
        isPlaying = true;
        return m_orchestra.play();
    }

    public StatusCode stop() {
        if(isPlaying) NCDebug.Debug.debug("Orchestra: stop");
        isPlaying = false;
        return m_orchestra.stop();
    }

    public StatusCode pause() {
        NCDebug.Debug.debug("Orchestra: pause");
        isPlaying = false;
        return m_orchestra.pause();
    }

    public NCOrchestra withMusic(String music) {
        if (m_music.compareTo(music) != 0) {
            m_music = music;
            NCDebug.Debug.debug("Orchestra: song change "+music);
            flagChanged = true;
        }
        update();

        return this;
    }

    private StatusCode addInstruments(TalonFX... motors) {
        StatusCode retErr = StatusCode.OK;

        // Iterate over drive motors
        for(var motor : motors) {
            var err = m_orchestra.addInstrument(motor);

            if (err.isError()) {
                retErr = err;
            } else {
                NCDebug.Debug.debug("Orchestra: Instrument added "+motor.getDeviceID());
            }
        }

        // Returns the last error code if an error has occurred
        return retErr;
    }

    // Songs
    // music/Brawl-Theme.chrp
    // music/Megalovania.chrp
    // music/Rickroll.chrp
    // music/Still-Alive.chrp

    // How to Use it
    // dj.google().onTrue(new InstantCommand(() -> {
    //     if(m_orchestra.isPlaying()) {
    //       m_orchestra.stop();
    //     } else {
    //       m_orchestra.withMusic("music/Still-Alive.chrp").play();
    //     }
    //   }).ignoringDisable(true));
}