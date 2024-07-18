const anchorsRanging: Record<string, {to: string, distance: string}> = {}

export const setAnchorRanging = (a: string, b: string, distance: string) => {
    anchorsRanging[a] = {to: b, distance}
    anchorsRanging[b] = {to: a, distance}
}

export const getAnchorRanging = (a: string, b: string) => {
    console.log(JSON.stringify(anchorsRanging, null, 2));
}
