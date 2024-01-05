# Devcontainer

This folder contains all files needed for the development container:

1. devcontainer.json and postCreateCommand.sh: Instructions for VS Code on how to create the dev container based on
  the previously created image
2. .vscode: The VS Code settings folder. To be used in the devcontainer to create a out of the box working Intellisense

## Remote Host

If you want to use a devcontainer on a remote host add the following to the settings.json of your VS Code config in your
local workspace and replace the placeholder with your values:

```json
"docker.host": "ssh://your-remote-user@your-remote-machine-fqdn-or-ip-here",
```

Also update the workspace mount to a volume in the [devcontainer.json](devcontainer.json) file:

```json
"workspaceMount": "source=seerep-ws,target=/home/docker/workspace/src,type=volume",
```

Make sure that your SSH agent is running and knows about the keys needed to connect to the remote host. (Following
instructions copied from [here](https://code.visualstudio.com/docs/remote/containers#_using-ssh-keys))

1. Adding keys to the SSH agent: `ssh-add $HOME/.ssh/github_rsa`
2. Starting the SSH Agent once: `eval "$(ssh-agent -s)"`
3. First, start the SSH Agent in the background by running the following in a terminal: `eval "$(ssh-agent -s)"`
4. Then add these lines to your `~/.bash_profile` or `~/.zprofile` (for Zsh) so it starts on login:

```bash
if [ -z "$SSH_AUTH_SOCK" ]; then
  # Check for a currently running instance of the agent
  RUNNING_AGENT="`ps -ax | grep 'ssh-agent -s' | grep -v grep | wc -l | tr -d '[:space:]'`"
  if [ "$RUNNING_AGENT" = "0" ]; then
      # Launch a new instance of the agent
      ssh-agent -s &> $HOME/.ssh/ssh-agent
  fi
  eval `cat $HOME/.ssh/ssh-agent`
fi
```

Some information regarding devcontainers:

- <https://code.visualstudio.com/docs/remote/containers>
- <https://code.visualstudio.com/docs/remote/create-dev-container>
- <https://code.visualstudio.com/docs/remote/containers-advanced>
- <https://code.visualstudio.com/docs/remote/containers-advanced#_developing-inside-a-container-on-a-remote-docker-host>
- <https://code.visualstudio.com/docs/remote/devcontainerjson-reference>
