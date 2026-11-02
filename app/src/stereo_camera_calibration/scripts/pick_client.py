#!/usr/bin/env python3
"""Run ball picks from a terminal: Enter = one pick, q = quit.

Calls /pick_ball and waits for the final state on /pick_status. With the gentle
closing (gripper target -0.008), a held ball and an empty gripper both end near
-0.005 to -0.008: check the ball visually.
"""
import json
import sys
import time

import rclpy
from std_srvs.srv import Trigger


def call(node, client, timeout=10.0):
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def pick(node, pick_client, status_client):
    response = call(node, pick_client)
    if response is None or not response.success:
        print('Prise refusée :', response.message if response else 'pas de réponse')
        return
    print(response.message)
    operation = None
    start = time.time()
    while time.time() - start < 120:
        status = call(node, status_client)
        data = json.loads(status.message) if status else {}
        operation = operation or data.get('operation_id')
        if data.get('state') in ('succeeded', 'failed') and data.get('operation_id') == operation:
            gripper = data.get('joints', {}).get('gripper_left_joint')
            print(f"{'Terminée' if data['state'] == 'succeeded' else 'ÉCHEC'} en {time.time() - start:.0f} s : "
                  f"{data.get('message')}")
            if gripper is not None:
                print(f'-> pince {gripper:+.4f}')
            return
        time.sleep(0.5)
    print('Pas de résultat après 120 s.')


def main():
    rclpy.init()
    node = rclpy.create_node('pick_client')
    pick_client = node.create_client(Trigger, '/pick_ball')
    status_client = node.create_client(Trigger, '/pick_status')
    if not pick_client.wait_for_service(timeout_sec=10):
        sys.exit('Service /pick_ball introuvable : lancer ball_pick_hardware.launch.py.')
    try:
        while True:
            answer = input('\nEntrée = prise, q = quitter : ').strip().lower()
            if answer == 'q':
                break
            pick(node, pick_client, status_client)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
