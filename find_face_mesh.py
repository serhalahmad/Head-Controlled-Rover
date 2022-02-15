import math

import cv2
import numpy as np
import mediapipe as mp


class FaceMeshDetector:
    def __init__(self, static_mode=False, min_detection_con=0.5, min_tracking_con=0.5):
        self.i = 0
        self.blink = 0
        self.blinked = False
        self.face = []
        self.static_mode = static_mode
        self.min_detection_con = min_detection_con
        self.min_tracking_con = min_tracking_con
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(static_image_mode=self.static_mode, min_detection_confidence=self.min_detection_con,
                                                    min_tracking_confidence=self.min_tracking_con)
        self.draw_spec = self.mp_draw.DrawingSpec(thickness=1, circle_radius=1)

    def find_face_mesh(self, frame, draw: bool = True):
        """
        :param frame:
        :param draw:
        :return:
        """
        self.frame = frame
        self.img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.face_mesh.process(self.img_rgb)
        self.face = []
        self.ih, self.iw, self.ic = frame.shape
        if self.results.multi_face_landmarks:
            if draw:
                self.mp_draw.draw_landmarks(frame, self.results.multi_face_landmarks[0], self.mp_face_mesh.FACEMESH_CONTOURS, self.draw_spec,
                                            self.draw_spec)
            if self.results.multi_face_landmarks[0].landmark:
                self.face = self.results.multi_face_landmarks[0].landmark
            # for id, lm in enumerate(self.results.multi_face_landmarks[0].landmark):
            #     # print(lm)
            #     ih, iw, ic = frame.shape
            #     x, y = int(lm.x * iw), int(lm.y * ih)
            #     # cv2.putText(frame, str(id), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
            #     # print(x, y)
            #     self.face.append([x, y])

    def get_head_position(self, thresh):
        face_3d = []
        face_2d = []
        action = 'nothing'
        direction = 'center'
        for idx, lm in enumerate(self.face):
            if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                if idx == 1:
                    nose_2d = (lm.x * self.iw, lm.y * self.ih)

                x, y = int(lm.x * self.iw), int(lm.y * self.ih)
                face_2d.append([x, y])
                face_3d.append([x, y, lm.z])

        if len(face_2d) > 0:
            face_2d = np.array(face_2d, dtype=np.float64)
            face_3d = np.array(face_3d, dtype=np.float64)

            focal_length = 1 * self.iw

            cam_matrix = np.array([[focal_length, 0, self.ih / 2],
                                   [0, focal_length, self.iw / 2],
                                   [0, 0, 1]])

            dist_matrix = np.zeros((4, 1), dtype=np.float64)

            success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

            rot_matrix = cv2.Rodrigues(rot_vec)[0]
            angles = cv2.RQDecomp3x3(rot_matrix)[0]

            x = angles[0] * 360
            y = angles[1] * 360

            if x < 0:
                action = 'forward'
                if y < -thresh:
                    direction = 'right'
                elif y > thresh:
                    direction = 'left'
                else:
                    direction = 'center'
            elif x > thresh:
                action = 'backward'
                if y < -thresh:
                    direction = 'right'
                elif y > thresh:
                    direction = 'left'
                else:
                    direction = 'center'
            else:
                action = 'nothing'
                if y < -thresh:
                    direction = 'right'
                elif y > thresh:
                    direction = 'left'
                else:
                    direction = 'center'
            print('Action:\t', action)
            print('Direction:\t', direction)
            print('\n')
            p1 = (int(nose_2d[0]), int(nose_2d[1]))
            p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))
            cv2.line(self.frame, p1, p2, (255, 0, 0), 3)
        return action, direction

    def blink_detector(self, frame):
        """
        :param frame:
        :return:
        """
        thresh = 4.3
        open = thresh - 0.1
        rh_right = self.face[33]
        rh_left = self.face[133]
        rv_top = self.face[159]
        rv_bottom = self.face[145]
        # cv2.line(frame, rh_right, rh_left, (0, 255, 0), 1)
        # cv2.line(frame, rv_top, rv_bottom, (255, 255, 255), 1)
        lh_right = self.face[263]
        lh_left = self.face[362]
        lv_top = self.face[386]
        lv_bottom = self.face[374]
        # cv2.line(frame, lh_right, lh_left, (0, 255, 0), 1)
        # cv2.line(frame, lv_top, lv_bottom, (255, 255, 255), 1)
        rh_distance = euclidean_distance(rh_right, rh_left)
        rv_distance = euclidean_distance(rv_top, rv_bottom)
        lv_distance = euclidean_distance(lv_top, lv_bottom)
        lh_distance = euclidean_distance(lh_right, lh_left)
        re_ratio = rh_distance / rv_distance
        le_ratio = lh_distance / lv_distance
        cv2.putText(frame, f'Blinks: {int(self.blink)}', (20, 70), cv2.FONT_HERSHEY_PLAIN,
                    3, (0, 255, 0), 3)
        if re_ratio > thresh and le_ratio > thresh and not self.blinked:
            self.blinked = True
            self.blink += 1
        elif re_ratio > thresh and le_ratio < open:
            self.blinked = False
            cv2.putText(frame, 'Right Wink', (20, 110), cv2.FONT_HERSHEY_PLAIN,
                        3, (0, 255, 0), 3)
        elif le_ratio > thresh and re_ratio < open:
            self.blinked = False
            cv2.putText(frame, 'Left Wink', (20, 110), cv2.FONT_HERSHEY_PLAIN,
                        3, (0, 255, 0), 3)
        else:
            self.blinked = False
        # print(re_ratio, le_ratio)
        return self.blink

    def get_eyes_position(self, frame, thresh):
        right_eye = frame.copy()
        left_eye = frame.copy()
        right_eye = right_eye[int(self.face[30].y * self.ih):int(self.face[26].y * self.ih),
                    int(self.face[156].x * self.iw):int(self.face[188].x * self.iw)]
        left_eye = left_eye[int(self.face[286].y * self.ih):int(self.face[254].y * self.ih),
                   int(self.face[399].x * self.iw):int(self.face[265].x * self.iw)]
        right_eye = cv2.resize(right_eye, (int(right_eye.shape[1] * 4), int(right_eye.shape[0] * 4)), interpolation=cv2.INTER_LINEAR)
        left_eye = cv2.resize(left_eye, (int(left_eye.shape[1] * 4), int(left_eye.shape[0] * 4)), interpolation=cv2.INTER_LINEAR)
        right_eye_th = cv2.cvtColor(right_eye, cv2.COLOR_RGB2GRAY)
        left_eye_th = cv2.cvtColor(left_eye, cv2.COLOR_RGB2GRAY)
        cv2.line(right_eye, (int(right_eye.shape[1] / 2), 0), (int(right_eye.shape[1] / 2), right_eye.shape[0]), (0, 0, 0), 1)
        cv2.line(left_eye, (int(left_eye.shape[1] / 2), 0), (int(left_eye.shape[1] / 2), left_eye.shape[0]), (0, 0, 0), 1)
        cv2.line(right_eye, (0, int(right_eye.shape[0] / 2)), (right_eye.shape[1], int(right_eye.shape[0] / 2)), (0, 0, 0), 1)
        cv2.line(left_eye, (0, int(left_eye.shape[0] / 2)), (left_eye.shape[1], int(left_eye.shape[0] / 2)), (0, 0, 0), 1)
        right_eye_th = cv2.threshold(right_eye_th, thresh, 255, cv2.THRESH_BINARY_INV)[1]
        left_eye_th = cv2.threshold(left_eye_th, thresh, 255, cv2.THRESH_BINARY_INV)[1]
        right_eye_opening = cv2.morphologyEx(right_eye_th, cv2.MORPH_OPEN, (5, 5))
        left_eye_opening = cv2.morphologyEx(left_eye_th, cv2.MORPH_OPEN, (5, 5))
        right_eye_dilation = cv2.erode(right_eye_opening, (5, 5), iterations=5)
        left_eye_dilation = cv2.erode(left_eye_opening, (5, 5), iterations=5)
        l_out = np.zeros((70, 170, 3))
        r_out = np.zeros((70, 170, 3))
        m = cv2.moments(right_eye_dilation)
        r_x = int(m["m10"] / m["m00"])
        m = cv2.moments(left_eye_dilation)
        l_x = int(m["m10"] / m["m00"])
        print('Right Eye:')
        if abs(r_x - int(right_eye.shape[1] / 2)) < 10:
            cv2.putText(r_out, 'Center', (0, int(r_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
            print('\tCenter')
        else:
            if r_x > int(right_eye.shape[1] / 2):
                cv2.putText(r_out, 'Left', (0, int(r_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
                print('\tLeft')
            else:
                cv2.putText(r_out, 'Right', (0, int(r_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
                print("\tRight")
        print('Left Eye:')
        if abs(l_x - int(left_eye.shape[1] / 2)) < 10:
            cv2.putText(l_out, 'Center', (0, int(l_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
            print('\tCenter')
        else:
            if l_x > int(left_eye.shape[1] / 2):
                cv2.putText(l_out, 'Left', (0, int(l_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
                print('\tLeft')
            else:
                cv2.putText(l_out, 'Right', (0, int(l_out.shape[0] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0))
                print("\tRight")
        cv2.imshow('Right Eye', right_eye)
        cv2.imshow('Right Eye Detection', right_eye_dilation)
        cv2.imshow('Right Eye Output', r_out)
        cv2.imshow('Left Eye', left_eye)
        cv2.imshow('Left Eye Detection', left_eye_dilation)
        cv2.imshow('Left Eye Output', l_out)
        return right_eye, r_out, right_eye_dilation, left_eye, l_out, left_eye_dilation


def euclidean_distance(rh_right, rh_left):
    """
    calculates euclidean distance between right and left points on same axis
    :param rh_right: right position
    :param rh_left: left position
    :return: distance
    """
    x, y = rh_right
    x1, y1 = rh_left
    distance = math.sqrt((x1 - x) ** 2 + (y1 - y) ** 2)
    return distance

