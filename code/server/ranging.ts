const anchorsRanging: Record<string, {to: string, distance: string}> = {}

export const setRanging = (a: string, b: string, distance: string) => {
    anchorsRanging[a] = {to: b, distance}
    anchorsRanging[b] = {to: a, distance}
}

export const getRanging = () => {
    
}
