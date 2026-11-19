#pragma once

#include "Components/LineBatchComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/World.h"

// Set a per-view exclusion, not the world's component visibility. This is
// honored by automatic PIP captures, deferred RPC captures and manual captures.
// HideComponent uses AddUnique; refreshing the filter does not grow the list.
inline void ApplyAirSimCaptureVisibility(USceneCaptureComponent2D* capture)
{
    if (capture == nullptr)
        return;
    UWorld* world = capture->GetWorld();
    if (world == nullptr)
        return;

    // A cube face at the body origin must not image the carrying drone.
    // This is a per-capture filter; the vehicle remains visible in the UE window
    // and in every other vehicle's sensors.
    AActor* camera = capture->GetOwner();
    AActor* vehicle = camera ? camera->GetAttachParentActor() : nullptr;
    if (vehicle != nullptr)
        capture->HideActorComponents(vehicle);

    capture->HideComponent(world->LineBatcher);
    capture->HideComponent(world->PersistentLineBatcher);
    capture->HideComponent(world->ForegroundLineBatcher);
}
