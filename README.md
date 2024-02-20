# Aimmy V2
![image](https://github.com/IceyFL/Undetected-Aimmy/assets/82657910/16805e52-88a4-49a9-988b-71d1faeee0aa)

> [!NOTE]
> If you enjoy Aimmy, please consider giving us a star ⭐! We appreciate it! :)

<div align="center">
  
  <p>
    <a href="https://aimmy.dev/" target="_blank">
      <img width="100%" src="https://raw.githubusercontent.com/Babyhamsta/Aimmy/master/readme_assets/AimmyV2Banner.png"></a>
  </p>

Aimmy is a multi-functional AI-based Aim Aligner that was developed by BabyHamsta & Nori for the purposes of making gaming more accessible for a wider audience.

Unlike most products of a similar caliber, Aimmy utilizes DirectML, ONNX, and YOLOV8 to detect players and is written in C# instead of Python. This makes it incredibly fast and efficient, and is one of the only AI Aim Aligners that runs well on AMD GPUs, which would not be able to use hardware acceleration otherwise due to the lack of CUDA support. 

Aimmy also has a myriad of [features](#features) that sets itself apart from other AI Aim Aligners, like the ability to hotswap models, and settings that allow you to adjust your aiming accuracy. 

Aimmy is 100% free to use, ad-free, and is actively not for profit. Aimmy is not, and will never be for sale, and is considered a **source-available** product as **we actively discourage other developers from making profit-focused forks of Aimmy**.

Join our Discord: https://discord.gg/Aimmy

If you want to show Aimmy to your friends, send them to our website: https://aimmy.dev/

## Table of Contents
[What is the purpose of Aimmy?](#what-is-the-purpose-of-aimmy) | [How does Aimmy Work?](#how-does-aimmy-work) | [Features](#features) | [Setup](#setup) | [How is Aimmy better than similar AI-Based tools?](#how-is-aimmy-better-than-similar-ai-based-tools) | [What is the Web Model?](#what-is-the-web-model) | [How do I train my own model?](#how-do-i-train-my-own-model) | [How do I upload my model to the "Downloadable Models" menu](#how-do-i-upload-my-model-to-the-downloadable-models-menu)



## What is the purpose of Aimmy?
### Aimmy was designed for Gamers who are at a severe disadvantage over normal gamers.
### This includes but is not limited to:
- Gamers who are physically challenged
- Gamers who are mentally challenged
- Gamers who suffer from untreated/untreatable visual impairments
- Gamers who do not have access to a seperate Human-Interface Device (HID) for controlling the pointer
- Gamers trying to improve their reaction time
- Gamers with poor Hand/Eye coordination
- Gamers who perform poorly in FPS games
- Gamers who play for long periods in hot environments, causing greasy hands that make aiming difficult 

## How does Aimmy Work?
Aimmy works by using AI Image Recognition to detect opponents, pointing the player towards the direction of an opponent accordingly. 

The gamer is now left to perform any actions they believe is necessary.

Additionally, a Gamer that uses Aimmy is also given the option to turn on Auto-Trigger. Auto-Trigger relieves the need to repeatedly tap the HID to shoot at a player. This is especially useful for physically challenged users who may have trouble with this action.

## Features
- AI Aim Aligning
- Aim Keybind Switching
- Predictions
- Bezier Curve Movement + Fake Jitter
- Undetection Launcher
- Adjustable FOV, Mouse Sensitivity, X Axis, Y Axis, and Model Confidence
- Auto Trigger and Trigger Delay
- Hot Model Swapping (No need to reload application)
- Hot Config Swapping (switch between presets easily)
- [Downloadable Model System](#how-do-i-upload-my-model-to-the-downloadable-models-menu)
- Image capture while playing (For labeling to further AI training)

## Setup
- Download and Install the x64 version of [.NET Runtime 8.0.X.X](https://dotnet.microsoft.com/en-us/download/dotnet/thank-you/runtime-desktop-8.0.2-windows-x64-installer)
- Download and Install the x64 version of [Visual C++ Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe)
- Download Aimmy from [Releases](https://github.com/BabyHamsta/Aimmy/releases) (Make sure it's the Aimmy zip and not Source zip)
- Extract the Aimmy.zip file
- Run Aimmy.exe
- Choose your Model and Enjoy :)

</div>
