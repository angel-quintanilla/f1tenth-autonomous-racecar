# SETUP
This Repo is meant to be cloned into your ``~/ws/gym-one/`` directory, as the ``f1tenth_gym_ros`` directory has machine specific paths in the ``docker-compose.yml`` file, so we'll avoid that headache :). 

## Cloning a Private Repo 
If you don't have your github account tied to your Linux machine, you'll have to tie it with an SSH Key, or Authentication Token: 
- Authentication Token : https://stackoverflow.com/questions/68775869/message-support-for-password-authentication-was-removed
- SSH Key(Preferred)   : https://www.youtube.com/watch?v=5jwzAhcovMU

## After Cloning into ``~/ws/gym-one``
As mentioned, ``ws/gym-one/f1tenth_gym_ros/docker-compose.yml`` will need to be updated to include the directory, update volumes with these 
```
  - /your/path/to/ws/gym-one/f1tenth-cs489/safety_node/:/sim_ws/src/safety_node
  - /your/path/to/ws/gym-one/f1tenth-cs489/wall_follow/:/sim_ws/src/wall_follow
  - /your/path/to/ws/gym-one/f1tenth-cs489/gap_follow/:/sim_ws/src/gap_follow
  - /your/path/to/ws/gym-one/f1tenth-cs489/pure_pursuit/:/sim_ws/src/pure_pursuit
```

- Notice for your prior entries, such as safety_node and wall_follow, you just need to add the ``f1tenth-cs489/ directory`` in the middle there, 
- ``.:/sim_ws/src/f1tenth_gym_ros`` at line 29 should remain unchanged 


Now you're good to cook 👨‍🍳
