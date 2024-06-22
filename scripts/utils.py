def get_synced_images(image_queues, last_image_time, scales):
    # Create a dictionary to store the potential synchronized images
    potential_synced_images = {}

    # Iterate through all scales and their corresponding image queues
    for scale, queue in image_queues.items():
        for image in queue:
            timestamp = image.header.stamp
            if timestamp not in potential_synced_images:
                potential_synced_images[timestamp] = {}
            potential_synced_images[timestamp][scale] = image

    # Find all synchronized sets that have images from all scales
    synced_images = []
    for timestamp, images in potential_synced_images.items():
        if len(images) == len(scales):
            synced_images.append((timestamp, images))

    # Sort the synchronized images by timestamp
    synced_images.sort()

    # Find the first valid set of synchronized images
    for timestamp, images in synced_images:
        if last_image_time is None or timestamp > last_image_time:
            return [images[scale] for scale in scales], timestamp

    # If no valid synchronized images found, return None
    return None, last_image_time
