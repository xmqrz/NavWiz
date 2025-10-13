#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_audio_player.cfg.AudioPlayerConfig
import agv05_msgs.msg
import diagnostic_updater
import dynamic_reconfigure.server
import os
import pygame as pg
import random
import rospy
import std_msgs.msg


AC = agv05_msgs.msg.AudioControl
NUM_PLAYLISTS = 10


class AudioPlayer(object):

    def __init__(self):
        # setup pygame mixer
        freq = 44100     # audio CD quality
        bitsize = -16    # unsigned 16 bit
        channels = 2     # 1 is mono, 2 is stereo
        buffer = 2048    # number of samples (experiment to get best sound)
        pg.mixer.init(freq, bitsize, channels, buffer)

        # setup pygame mixer channels
        pg.mixer.set_reserved(0)
        pg.mixer.set_reserved(1)
        self.music_channel = pg.mixer.music
        self.alarm_channel = pg.mixer.Channel(0)
        self.beep_channel = pg.mixer.Channel(1)
        self.music_status = AC.STOP
        self.music_playlist = 0
        self.music_playlist_data = None
        self.music_error = None
        self.alarm_error = None
        self.beep_error = None
        self.resume_music_after_alarm = False

        # setup dynamic reconfigure server
        self.config = None
        self.dyncfg_server = dynamic_reconfigure.server.Server(agv05_audio_player.cfg.AudioPlayerConfig, self.handle_config)

        # setup diagnostic updater
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("AGV05")
        self.updater.add("Status", self.handle_diagnostic_status)

        # setup ROS subscribers
        audio_topic = 'agv05/audio/'
        rospy.Subscriber(audio_topic + 'music_control', AC, self.handle_music_control)
        rospy.Subscriber(audio_topic + 'music_volume', std_msgs.msg.Float32, self.handle_music_volume)
        rospy.Subscriber(audio_topic + 'alarm_control', AC, self.handle_alarm_control)
        rospy.Subscriber(audio_topic + 'alarm_volume', std_msgs.msg.Float32, self.handle_alarm_volume)
        rospy.Subscriber(audio_topic + 'beep_control', AC, self.handle_beep_control)
        rospy.Subscriber(audio_topic + 'beep_volume', std_msgs.msg.Float32, self.handle_beep_volume)

    def spin(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # check music playback
            if self.music_status == AC.PLAY and not pg.mixer.music.get_busy():
                self._load_next_music()

            self.updater.update()
            r.sleep()

    def handle_config(self, config, level):
        rospy.loginfo('Configuration received.')

        # disallow absolute path to prevent malicious file execution
        config.alarm_media_ = config.alarm_media_.strip('./')
        config.beep_media_ = config.beep_media_.strip('./')
        for i in range(1, 11):
            key = 'music_playlist_%d_files_' % i
            files = config[key].split(';')
            config[key] = ';'.join([f.strip('./') for f in files])

        # copy and compare configuration
        old_config = self.config
        self.config = config.copy()

        if not old_config or old_config.media_root_ != config.media_root_:
            self._load_alarm()
            self._load_beep()
            if self.music_status != AC.STOP and self._valid_playlist(self.music_playlist):
                self._load_music_playlist()
                self._load_next_music(play=(self.music_status == AC.PLAY))
        else:
            if old_config.alarm_media_ != config.alarm_media_:
                self._load_alarm()
            if old_config.beep_media_ != config.beep_media_:
                self._load_beep()
            if self.music_status != AC.STOP and self._valid_playlist(self.music_playlist):
                key = 'music_playlist_%d_' % self.music_playlist
                if old_config[key + 'files_'] != config[key + 'files_']:
                    self._load_music_playlist()
                    self._load_next_music(play=(self.music_status == AC.PLAY))

        self.music_channel.set_volume(self.config.music_volume_)
        self.alarm_channel.set_volume(self.config.alarm_volume_)
        self.beep_channel.set_volume(self.config.beep_volume_)
        return config

    def handle_music_control(self, msg):
        if self.music_status == msg.operation:
            if msg.operation == AC.PLAY and self._valid_playlist(msg.playlist):
                if self.music_playlist != msg.playlist:
                    self.music_playlist = msg.playlist
                    self._load_music_playlist()
                    self._load_next_music()
        else:
            if msg.operation == AC.PLAY and self._valid_playlist(msg.playlist):
                if self.music_status != AC.PAUSE or self.music_playlist != msg.playlist:
                    self.music_playlist = msg.playlist
                    self._load_music_playlist()
                    self._load_next_music()
                else:
                    if self.config.pause_music_on_alarm_ and self.alarm_channel.get_busy():
                        self.resume_music_after_alarm = True
                    else:
                        self.music_channel.unpause()
            elif msg.operation == AC.STOP:
                self.music_channel.stop()
                self.resume_music_after_alarm = False
            elif msg.operation == AC.PAUSE:
                self.music_channel.pause()
                self.resume_music_after_alarm = False
            else:
                return
            self.music_status = msg.operation

    def handle_music_volume(self, msg):
        self.dyncfg_server.update_configuration({'music_volume_': msg.data})

    def handle_alarm_control(self, msg):
        if msg.operation == AC.PLAY:
            if not self.alarm_channel.get_busy():
                self.alarm_channel.play(self.alarm_media, -1)
                if self.config.pause_music_on_alarm_ and self.music_status == AC.PLAY:
                    self.music_channel.pause()
                    self.resume_music_after_alarm = True
        elif msg.operation == AC.STOP:
            self.alarm_channel.stop()
            if self.resume_music_after_alarm:
                self.music_channel.unpause()
                self.resume_music_after_alarm = False

    def handle_alarm_volume(self, msg):
        self.dyncfg_server.update_configuration({'alarm_volume_': msg.data})

    def handle_beep_control(self, msg):
        if msg.operation == AC.PLAY:
            if not self.beep_channel.get_busy():
                self.beep_channel.play(self.beep_media)
        elif msg.operation == AC.STOP:
            self.beep_channel.stop()

    def handle_beep_volume(self, msg):
        self.dyncfg_server.update_configuration({'beep_volume_': msg.data})

    def handle_diagnostic_status(self, stat):
        if self.music_error or self.alarm_error or self.beep_error:
            stat.summary(diagnostic_updater.DiagnosticStatus.ERROR, 'Status Error')
        else:
            stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'Status OK')
        stat.add('Music', self.music_error if self.music_error else 'Ready')
        stat.add('Alarm', self.alarm_error if self.alarm_error else 'Ready')
        stat.add('Beep', self.beep_error if self.beep_error else 'Ready')
        return stat

    def _load_music_playlist(self):
        if not self._valid_playlist(self.music_playlist):
            return
        key = 'music_playlist_%d_' % self.music_playlist
        self.music_playlist_data = {
            'files': self.config[key + 'files_'].split(';'),
            'shuffle': self.config[key + 'shuffle_'],
            'loop': self.config[key + 'loop_'],
            'current': -1,
        }
        if self.music_playlist_data['shuffle']:
            random.shuffle(self.music_playlist_data['files'])

    def _load_next_music(self, play=True):
        playlist_data = self.music_playlist_data
        files = playlist_data['files']
        current = playlist_data['current'] + 1
        if current >= len(files):
            if playlist_data['loop']:
                current = 0
                if playlist_data['shuffle'] and len(files) > 2:
                    current_file = files[-1]
                    random.shuffle(files)
                    if files[0] == current_file:
                        i = random.randint(1, len(files) - 1)
                        files[0], files[i] = files[i], files[0]
            else:
                self.music_status = AC.STOP
                return
        playlist_data['current'] = current

        path = os.path.join(self.config.media_root_, playlist_data['files'][current])
        if os.path.exists(path):
            try:
                self.music_channel.load(path)
                if play:
                    if self.config.pause_music_on_alarm_ and self.alarm_channel.get_busy():
                        self.resume_music_after_alarm = True
                    else:
                        self.music_channel.play()
                self.music_error = None
            except pg.error as ex:
                self.music_error = 'Invalid music file: %s' % ex
        else:
            self.music_error = 'Invalid music path.'

    def _load_alarm(self):
        if not self.config.alarm_media_:
            self.alarm_error = 'Invalid media file.'
            return

        path = os.path.join(self.config.media_root_, self.config.alarm_media_)
        if os.path.exists(path):
            self.alarm_error = None
        else:
            self.alarm_error = 'Invalid media path.'
            return

        self.alarm_media = pg.mixer.Sound(path)

        if self.alarm_channel.get_busy():
            self.alarm_channel.play(self.alarm_media, -1)
        else:
            self.alarm_channel.stop()

    def _load_beep(self):
        if not self.config.beep_media_:
            self.beep_error = 'Invalid media file.'
            return
        path = os.path.join(self.config.media_root_, self.config.beep_media_)
        if os.path.exists(path):
            self.beep_error = None
        else:
            self.beep_error = 'Invalid media path.'
            return

        self.beep_media = pg.mixer.Sound(path)
        self.beep_channel.stop()

    def _valid_playlist(self, playlist):
        return isinstance(playlist, int) and playlist > 0 and playlist <= NUM_PLAYLISTS


# Main function
if __name__ == '__main__':
    rospy.init_node('agv05_audio_player', anonymous=False)
    rospy.loginfo('Node started.')
    player = AudioPlayer()
    player.spin()
